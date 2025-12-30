#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_ENABLED)

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <esp_idf_version.h>
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#include "esp_adc/adc_continuous.h"
#endif

#if (C_LOG_LEVEL <= LOG_INFO_NUM)
static const char * const _adc_battery_states_str[] = { ADC_BAT_STATES(STRINGIFY) };
static const char * const _adc_ulp_wake_sources_str[] = { ADC_ULP_WAKE_SOURCES(STRINGIFY_V) };
static const char * const _adc_ulp_adc_wake_reasons_str[] = { ADC_ULP_BAT_STATES(STRINGIFY_V) };
static const char * const _adc_ulp_button_wake_reasons_str[] = { ADC_ULP_BUTTON_WAKE_REASONS(STRINGIFY_V) };
const char * adc_battery_states_str(int i) { return _adc_battery_states_str[i]; };
const char * adc_ulp_wake_sources_str(int i) { return _adc_ulp_wake_sources_str[i]; };
const char * adc_ulp_adc_wake_reasons_str(int i) { return _adc_ulp_adc_wake_reasons_str[i]; };
const char * adc_ulp_button_wake_reasons_str(int i) { return _adc_ulp_button_wake_reasons_str[i]; };
#else
const char * nums[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };
const char * adc_battery_states_str(int i) {
#if (C_LOG_LEVEL <= LOG_ERR_NUM)
    if(i==ADC_BATTERY_CRITICAL_LOW)  return "CRITICAL_LOW";
#endif
    else return nums[i];
}
const char * adc_ulp_wake_sources_str(int i) { return nums[i]; }
const char * adc_ulp_adc_wake_reasons_str(int i) { return nums[i]; }
const char * adc_ulp_button_wake_reasons_str(int i) { return nums[i]; }
#endif

/* Regular ADC battery state tracking (when ULP is disabled) */
static uint32_t last_adc_reading_mv = 0;
static adc_battery_state_t last_adc_battery_state = ADC_BATTERY_NORMAL;

#define VOLTAGE_HISTORY_MS 1500U  // 1.5 second window for trend detection
#define TREND_DETECTION_WINDOW_MS 1000U  // 1 second to detect consistent trend
#define STABILIZATION_WINDOW_MS 500U   // 0.5 second to confirm voltage stabilizes
#define CHANGE_THRESHOLD_MV 100U        // 100mV minimum change for detection

// Use constants from adc_private.h instead of hardcoded values
static uint8_t history_count = 0;
static uint32_t last_charge_check_ms = 0;
static uint32_t last_charge_state_change_ms = 0;

// 2-phase detection system: trend detection + stabilization

static adc_battery_state_t pending_charge_event = ADC_BATTERY_NORMAL;
static uint32_t trend_detection_start_ms = 0;
static uint32_t stabilization_start_ms = 0;
static uint32_t trend_start_voltage_mv = 0;
static int32_t detected_trend_direction = 0;  // +1 for up, -1 for down, 0 for none
static bool trend_detection_active = false;
static bool stabilization_active = false;
static bool adc_initial_charging_state = false;
static bool adc_initial_sync_done = false;

// Charging state flag - moved from main module to ADC module as single source of truth
static bool adc_charging_is_on = false;

// LCD charge notification flag - set by ADC when charge events occur, cleared by LCD when processed
static volatile bool adc_lcd_charge_notification = false;

// Flag to force instant voltage reading after CHARGE_STOPPED event
static volatile bool force_instant_voltage = false;

/* Adaptive detection tunables */
/* Reduced multiplier to make dynamic threshold more sensitive in noisy conditions */
#define ADC_NOISE_MULTIPLIER     2   /* multiplier for MAD -> dynamic threshold (was 3) */
/* Require fewer consecutive confirmations to be more responsive in practice */
#define ADC_CONSEC_REQUIRED      2   /* require N consecutive detections to confirm (was 2) */

/* Consecutive confirmation counters (awake CPU-side) */
static uint8_t adc_consec_up = 0;
static uint8_t adc_consec_down = 0;
static int32_t adc_cumulative_positive = 0;
static uint32_t adc_cumulative_start_ms = 0;

ESP_EVENT_DEFINE_BASE(ADC_EVENT);

/* Battery monitoring integration with main application */
static void (*low_battery_callback)(void) = NULL;

/* Low battery monitoring state - managed by ADC timer */

// static float minimum_battery_voltage = 3.25f;  // Default, can be updated
#define LOW_BAT_SEQUENCE_TIME_MS (20 * 1000)  // 20 seconds in milliseconds

/* ADC event suppression during WiFi/GPS transitions */
static bool s_adc_events_suppressed = false;
static int64_t s_adc_suppression_start_time = 0;
#define ADC_SUPPRESSION_TIMEOUT_MS 5000  // 5 seconds max suppression


static const char * _adc_event_strings[] = { ADC_EVENT_LIST(STRINGIFY) };
const char * adc_event_strings(int id) {
    return id < lengthof(_adc_event_strings) ? _adc_event_strings[id] : "ADC_EVENT_UNKNOWN";
}

static const char *TAG = "adc";

#define V_GRAPH_LIPO_LEN 21
#define ADJ_LENGTH 24

adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;

/* Module-level cache populated by adc_update():
 * - s_cached_snapshot contains the most-recent adc_snapshot_t read by the
 *   periodic update. Other functions may read this to avoid repeated ULP reads.
 * - s_cached_snapshot_valid indicates whether the cache is populated.
 * - s_cached_batt_mv stores the calibrated battery millivolt value computed by
 *   the periodic update (reading). */
static adc_snapshot_t s_cached_snapshot = {0};
static bool s_cached_snapshot_valid = false;
static RTC_DATA_ATTR uint32_t s_cached_batt_mv = 0;

/* Accessors for other modules/functions to read the cached snapshot */
bool adc_get_cached_snapshot(adc_snapshot_t *out) {
    if (!out) return false;
    if (!s_cached_snapshot_valid) return false;
    *out = s_cached_snapshot;
    return true;
}

uint32_t adc_get_cached_batt_mv(void) {
    return s_cached_batt_mv;
}

static const uint16_t v_graph_lipo[V_GRAPH_LIPO_LEN] = {
    33000,  // 0
    36100,  // 5
    36900,  // 10
    37100,  // 15
    37300,  // 20
    37500,  // 25
    37700,  // 30
    37900,  // 35
    38000,  // 40
    38200,  // 45
    38400,  // 50
    38500,  // 55
    38700,  // 60
    39100,  // 65
    39500,  // 70
    39800,  // 75
    40200,  // 80
    40800,  // 85
    41100,  // 90
    41500,  // 95
    42000,  // 100
};

#define TIMEOUT_MAX portMAX_DELAY
static const TickType_t timeout_immediate = 0;
#define RESULT_MASK (RESULT_SIZE - 1)
#define MIN_READINGS 3

bool adc_lock(int timeout) {
    if (!adc_ctx.xMutex) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? TIMEOUT_MAX : 
                                     (timeout == 0) ? timeout_immediate : pdMS_TO_TICKS(timeout);
    return  xSemaphoreTake(adc_ctx.xMutex, timeout_ticks) == pdTRUE;
}

void adc_unlock() {
    if (adc_ctx.xMutex) {
        xSemaphoreGive(adc_ctx.xMutex);
    }
}

bool bat_safe_lock(int timeout) {
    if (!adc_ctx.batMutex) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? TIMEOUT_MAX :
                                     (timeout == 0) ? timeout_immediate : pdMS_TO_TICKS(timeout);
    return  xSemaphoreTake(adc_ctx.batMutex, timeout_ticks) == pdTRUE;
}

void bat_safe_unlock() {
    if (adc_ctx.batMutex) {
        xSemaphoreGive(adc_ctx.batMutex);
    }
}

#if !defined(CONFIG_LOGGER_ADC_MODE_ULP)
static inline void add_adc_reading(uint32_t value) {
    adc_ctx.adc_buffer.result[adc_ctx.adc_buffer.head] = value;
    adc_ctx.adc_buffer.head = (adc_ctx.adc_buffer.head + 1) & RESULT_MASK;
    adc_ctx.adc_buffer.count += (adc_ctx.adc_buffer.count < RESULT_SIZE);
}
#endif
#if defined(HAS_ADC_AVG)
static inline uint32_t get_recent_reading(uint8_t pos) {
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    return ULP_GET_U32(ulp_last_result);
#else
    return adc_ctx.adc_buffer.result[(adc_ctx.adc_buffer.head - 1 - pos) & RESULT_MASK];
#endif
}

static uint32_t get_recent_average_n(uint8_t num_readings) {
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    // ULP running_sum contains sum of raw ADC values, convert to average voltage
    uint32_t raw_sum = ULP_GET_U32(ulp_running_sum);
    uint32_t raw_avg = raw_sum >> ULP_ADC_HISTORY_SHIFT;  // Divide by history size (4)
    uint32_t sum = calibrate_adc_raw(raw_avg);  // Use calibration function
#else
    uint8_t available = adc_ctx.adc_buffer.count;
    
    // Clamp to available readings
    if (num_readings > available) {
        num_readings = available;
    }
    if (num_readings < 1) {
        return get_recent_reading(0); // Return current reading
    }
    
    uint32_t sum = 0;
    uint8_t idx = (adc_ctx.adc_buffer.head - 1) & RESULT_MASK;
    
    for (uint8_t i = 0; i < num_readings; i++) {
        sum += adc_ctx.adc_buffer.result[idx];
        idx = (idx - 1) & RESULT_MASK;
    }
#endif
    return sum / num_readings;
}

static uint32_t get_recent_average(void) {
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    // ULP running_sum contains sum of raw ADC values
    uint32_t raw_sum = ULP_GET_U32(ulp_running_sum);
    uint32_t raw_avg = raw_sum >> ULP_ADC_HISTORY_SHIFT;  // Divide by history size (4)
    return calibrate_adc_raw(raw_avg);  // Use calibration function
#else
    return get_recent_average_n(adc_ctx.adc_buffer.count);
#endif
}

static uint32_t get_progressive_average(void) {
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    uint8_t available = ULP_GET_U32(ulp_cycle_count) > ULP_ADC_HISTORY_SIZE ? ULP_ADC_HISTORY_SIZE : ULP_GET_U32(ulp_cycle_count);
    uint8_t max = ULP_ADC_HISTORY_SIZE;
#else
    uint8_t available = adc_ctx.adc_buffer.count;
    uint8_t max = RESULT_SIZE;
#endif
    if (available == 0) return 0;
    
    // Progressive stages based on buffer fill percentage
    uint8_t fill_percent = (available * 100) / max;
    
    if (fill_percent < 25) {
        // 0-25% filled: use current reading or tiny average
        return (available < 2) ? get_recent_reading(0) : get_recent_average_n(2);
    }
    else if (fill_percent < 50) {
        // 25-50% filled: use 25% of buffer size
        return get_recent_average_n(max / 4);
    }
    else if (fill_percent < 75) {
        // 50-75% filled: use 50% of buffer size  
        return get_recent_average_n(max / 2);
    }
    else if (fill_percent < 90) {
        // 75-90% filled: use 75% of buffer size
        return get_recent_average_n((max * 3) / 4);
    }
    else {
        // 90-100% filled: use 90% of buffer size (avoid very oldest readings)
        return get_recent_average_n((max * 9) / 10);
    }
}
#endif

adc_battery_state_t get_adc_state(void) {
    FUNC_ENTRY_ARGS(TAG," *** %s ***", adc_battery_states_str(last_adc_battery_state));
    return last_adc_battery_state;
}

/**
 * Common calibration function for both ULP and regular ADC readings
 * Applies hardware calibration if available, falls back to voltage conversion
 * @param raw_adc Raw ADC reading value
 * @return Calibrated voltage in millivolts, or 0 on error
 */
uint32_t calibrate_adc_raw(uint32_t raw) {
    if (raw == 0) {
        return 0; // Invalid reading
    }
    
    uint32_t calibrated_voltage = 0;
    
// #if defined(CONFIG_LOGGER_ADC_MODE_ULP)
//     // 4-point calibration for 3.2V-4.2V range
//     /* Direct calibration to battery voltage for 100k+100k divider */
    
//     if (raw < 1752) {
//         // 1752 raw → 3200mV actual (should be 2×1411=2822, but is 3200)
//         calibrated_voltage = (raw * 3200UL) / 1752;
//     }
//     else if (raw < 2128) {
//         // 2128 raw → 3801mV actual (should be 2×1806=3612, but is 3801)
//         calibrated_voltage = 3200 + ((raw - 1752) * 601UL) / 376;
//     }
//     else if (raw < 2242) {
//         // 2242 raw → 4001mV actual (should be 2×1907=3814, but is 4001)
//         calibrated_voltage = 3801 + ((raw - 2128) * 200UL) / 114;
//     }
//     else {
//         // 2367 raw → 4200mV actual (should be 2×2000=4000, but is 4200)
//         calibrated_voltage = 4001 + ((raw - 2242) * 199UL) / 125;
//     }

// #else
    /* Regular ADC Mode: Use hardware calibration if available */
    uint32_t mv = 0;
    if (raw_to_mv_wrapper((int)raw, &mv) == ESP_OK) {
        calibrated_voltage = mv;
    } else {
        ELOG(TAG, "ADC calibration not available, using manual conversion");
        calibrated_voltage = raw;
    }
// #endif
    FUNC_ENTRY_ARGSD(TAG, "raw=%lu, battery_mv=%lu", raw, calibrated_voltage);
    return calibrated_voltage;
}

/**
 * Convert raw ADC reading to calibrated voltage (in volts)
 * This function can be used with any raw ADC value, including stored RTC values
 */
static inline float adc_mv_to_voltage(uint32_t raw_adc_value) {
    return VOLTAGE_CONV_MV_TO_V(raw_adc_value); // Convert mV to V
}

/**
 * Common voltage validation function for both ULP and regular ADC
 * Validates voltage readings against board-specific thresholds
 * @param voltage_mv Voltage in millivolts
 * @param source_name Source description for logging ("ULP" or "ADC")
 * @return true if voltage is valid, false if likely pin conflict or unrealistic
 */
// bool validate_voltage_reading(uint32_t voltage_mv) {
//     // T5 and other boards: More conservative voltage range
//     if (voltage_mv > USB_VOLTAGE_MAX_MV) {  // Above realistic max with battery connected
//         WLOG(TAG, "Voltage %lu mV exceeds expected range - pin conflict detected", voltage_mv);
//         return false;
//     }
//     if (voltage_mv < BATTERY_VOLTAGE_MIN_MV) {   // Below realistic operating voltage
//         WLOG(TAG, "Voltage %lu mV below realistic operating range", voltage_mv);
//         return false;
//     }
// #if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))    
//     // Log charging detection for T-Display S3
//     if (voltage_mv > BATTERY_HIGH_MV) {
//         DLOG(TAG, "%s charging detected: %lu mV (USB connected)", source_name, voltage_mv);
//     }
// #endif
//     return true;
// }

/**
 * Helper function for voltage validation and clamping
 * Consolidates duplicated voltage validation logic for different board types
 */
float validate_and_clamp_voltage_mv(uint32_t voltage_mv) {
    uint32_t result;
    if (voltage_mv > USB_VOLTAGE_MAX_MV) goto fallback;
    if (voltage_mv < BATTERY_VOLTAGE_MIN_MV) goto fallback;
    if (voltage_mv > CHARGING_VOLTAGE_THRESHOLD_MV) { /* ... */ }
    goto end;
fallback:
    return VOLTAGE_CONV_MV_TO_V(FALLBACK_VOLTAGE_MV);
end:
    return VOLTAGE_CONV_MV_TO_V(voltage_mv);
}

/**
 * Enhanced battery state event posting with app mode context
 * Handles posting ESP events for battery state changes with intelligent filtering
 */
static void post_battery_state_event(adc_battery_state_t state, const char* source) {
    FUNC_ENTRY_ARGS(TAG, "source: %s, state: %d", source, state);
    uint32_t voltage_mv = adc_get_cached_batt_mv();

    // Check if we should filter events based on app mode
    if (should_filter_charge_events()) {
        // During boot/shutdown - only allow critical events to pass through
        if (state != ADC_BATTERY_CRITICAL_LOW) {
            WLOG(TAG, "%s charge event filtered during app mode transition: state=%d", source, state);
            return;
        }
    }
    
    // Check event suppression (WiFi transitions, etc.)
    int32_t event_id;
    switch (state) {
        case ADC_BATTERY_LOW:
            event_id = ADC_EVENT_BATTERY_LOW;
            break;
        // case ADC_BATTERY_HIGH:
        //     event_id = ADC_EVENT_BATTERY_HIGH; 
        //     break;
        case ADC_BATTERY_CHARGING_STARTED:
            event_id = ADC_EVENT_CHARGE_STARTED;
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
            event_id = ADC_EVENT_CHARGE_STOPPED;
            break;
        case ADC_BATTERY_CHARGE_STABILIZED:
            event_id = ADC_EVENT_CHARGE_STABILIZED;
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            event_id = ADC_EVENT_BATTERY_CRITICAL;
            break;
        default:
            // Normal state - no event needed
            return;
    }
    
    // Check if this specific event should be suppressed
    if (adc_should_suppress_event(event_id)) {
        ILOG(TAG, "%s event suppressed during system transition: %s", source, adc_event_strings(event_id));
        return;
    }
    
    // Additional charge state consistency logic
    if (state == ADC_BATTERY_CHARGING_STARTED || state == ADC_BATTERY_CHARGING_STOPPED) {
        bool current_charging = get_current_charging_state();
        bool new_charging = (state == ADC_BATTERY_CHARGING_STARTED);
        
        // Prevent redundant charge state changes
        if (current_charging == new_charging) {
            ILOG(TAG, "%s redundant charge state change filtered: already %s", source,
                 new_charging ? "charging" : "not charging");
            return;
        }
        
        // Update internal ADC state to reflect the change we're about to post
        last_adc_battery_state = state;
        
        // Sync charging state to ULP
// #if defined(CONFIG_ULP_COPROC_ENABLED)
//         bool charging = (state == ADC_BATTERY_CHARGING_STARTED);
//         ULP_SET_U32(ulp_charging_active, charging ? 1 : 0);
// #endif
    }
    
    // Post the event
    switch (state) {
        case ADC_BATTERY_LOW:
            WLOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_LOW), voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_LOW, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_HIGH:
            ILOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_HIGH), voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_HIGH, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STARTED:
            ILOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_CHARGING_STARTED), voltage_mv);
            adc_charging_is_on = true;  // Update internal flag
            adc_lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STARTED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
            ILOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_CHARGING_STOPPED), voltage_mv);
            adc_charging_is_on = false;  // Update internal flag
            adc_lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STOPPED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGE_STABILIZED:
            ILOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_CHARGE_STABILIZED), voltage_mv);
            adc_charging_is_on = true;  // Keep charging flag active
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STABILIZED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            ELOG(TAG, "%s detected %s: %ld mV", source, adc_battery_states_str(ADC_BATTERY_CRITICAL_LOW), voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_CRITICAL, NULL, 0, pdMS_TO_TICKS(100));
            break;
        default:
            // Normal state - no event needed
            break;
    }
}

// Configuration - tune these for your battery characteristics
typedef struct {
    uint16_t peak_threshold_percent;  // 15% = peak
    uint16_t drift_threshold_mv;      // 200mV normal drift
    uint16_t trend_window;           // 4 readings for trend
    uint16_t charge_threshold_mv;    // 100mV for charge detection
    uint16_t stabilization_ms;       // 500ms stabilization
} adc_config_t;

static const adc_config_t adc_cfg = {
    .peak_threshold_percent = 15,
    .drift_threshold_mv = 200,
    .trend_window = 4,
    .charge_threshold_mv = 100,
    .stabilization_ms = 500
};
typedef enum {
    TREND_STABLE = 0,
    TREND_RISING,
    TREND_FALLING,
    TREND_RAPID_RISING,
    TREND_RAPID_FALLING,
    TREND_VOLATILE
} voltage_trend_t;

// Unified analysis result
typedef struct {
    uint32_t filtered_reading;    // Peak-filtered value
    voltage_trend_t trend;        // Current trend
    bool is_stable;              // Reading is stable
    int32_t rate_of_change;      // mV/reading trend
    bool is_charging_event;      // Potential charge state change
    uint32_t noise_mad;          // Mean Absolute Deviation of recent readings (raw units)
    uint32_t dyn_threshold;      // Dynamic threshold (raw units)
} adc_analysis_t;

/**
 * Elegant single-pass analysis
 * - Peak filtering
 * - Trend detection  
 * - Stability analysis
 * - Charge event detection
 */
static adc_analysis_t analyze_adc_readings(uint32_t current_reading, uint8_t available);

/* Helper: compute MAD (mean absolute deviation) over available history.
 * Uses ULP snapshot when available for efficiency; falls back to CPU computation.
 */
static uint32_t compute_history_mad(uint8_t available)
{
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    uint32_t cycle_cnt = ULP_GET_U32(ulp_cycle_count);
    uint8_t hist_count = (cycle_cnt > ULP_ADC_HISTORY_SIZE) ? ULP_ADC_HISTORY_SIZE : (uint8_t)cycle_cnt;
    if (hist_count == 0) hist_count = 1;

    adc_snapshot_t snap = {0};
    bool have_snap = adc_get_cached_snapshot(&snap);
    if (!have_snap || !SNAPSHOT_HAS_MAD(&snap)) {
        have_snap = adc_snapshot_take(&snap, true, 3);
    }
    if (have_snap && SNAPSHOT_HAS_MAD(&snap)) {
        return snap.mad;
    }
    /* If snapshot failed, fall back to safe single-sample estimate */
    return 0;
#else
    uint8_t hist_count = (available == 0) ? 1 : available;
    uint32_t hist_sum = 0;
    for (uint8_t i = 0; i < hist_count; i++) {
        hist_sum += get_recent_reading(i);
    }
    uint32_t hist_avg = hist_sum / hist_count;
    uint32_t mad = 0;
    for (uint8_t i = 0; i < hist_count; i++) {
        uint32_t v = get_recent_reading(i);
        mad += (v > hist_avg) ? (v - hist_avg) : (hist_avg - v);
    }
    mad /= hist_count;
    return mad;
#endif
}

/* Unified ADC snapshot API implementation. See prototype in adc_private.h */
bool adc_snapshot_take(adc_snapshot_t *out, bool compute_mad, int max_retries)
{
    FUNC_ENTRYD(TAG);
    if (!out) return false;
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    /* Take a fresh snapshot of current ULP state, then read it back */
    if (!adc_ulp_take_history_snapshot(0)) {  /* snapshot_state = 0 for manual snapshots */
        out->valid_count = 0;
        SNAPSHOT_CLEAR_MAD_FLAG(out);
        SNAPSHOT_CLEAR_STATE(out);
        return false;
    }
    /* Use ULP snapshot reader to read the snapshot we just took */
    return ulp_history_snapshot_take((ulp_history_snapshot_t*)out, compute_mad, max_retries);
#else
    uint8_t available = adc_ctx.adc_buffer.count;
    if (available == 0) {
        out->valid_count = 0;
        out->has_mad = false;
        return false;
    }
    out->valid_count = available;
    uint32_t sum = 0;
    uint8_t idx = (adc_ctx.adc_buffer.head - 1) & RESULT_MASK;
    for (uint8_t i = 0; i < available; i++) {
        uint32_t v = adc_ctx.adc_buffer.result[idx];
        sum += v;
        idx = (idx - 1) & RESULT_MASK;
    }
    out->running_sum = sum;
    out->cycle_count = available;
    out->history_idx = adc_ctx.adc_buffer.head & RESULT_MASK;
    out->history_avg = sum / available;
    out->last_sample = get_recent_reading(0);
    SNAPSHOT_CLEAR_STATE(out);
    out->snapshot_state = 0;
    if (compute_mad) {
        uint32_t mad = 0;
        idx = (adc_ctx.adc_buffer.head - 1) & RESULT_MASK;
        for (uint8_t i = 0; i < available; i++) {
            uint32_t v = adc_ctx.adc_buffer.result[idx];
            mad += (v > out->history_avg) ? (v - out->history_avg) : (out->history_avg - v);
            idx = (idx - 1) & RESULT_MASK;
        }
        out->mad = mad / available;
        out->has_mad = true;
    } else {
        out->has_mad = false;
    }
    return true;
#endif
}

#define MV_TO_RAW(mv) ((mv * 1752UL) / 3200UL)
typedef struct {
    uint32_t threshold;
    int16_t rate_of_change;
} threshold_entry_t;

typedef struct {
    uint8_t readings;
    int16_t threshold;
} trend_entry_t;

#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
const threshold_entry_t rise_thresh[] = {
    {1971L, 82L},   // Below 3600mV equivalent 150mv in raw = (150 × 1752) / 3200 = 82 raw
    {2081L, 38L},   // 3600-3800mV equivalent 70mv in raw = (70 × 1752) / 3200 = 38 raw
    {2190L, 27L},   // 3800-4000mV equivalent 50mv in raw = (50 × 1752) / 3200 = 27 raw
    {UINT32_MAX, 22L},    // Above 4000mV equivalent 40mv in raw = (40 × 1752) / 3200 = 22 raw
    {2217L, INT16_MAX},    // Additional threshold for high battery 4050mV in raw = (4050 × 1752) / 3200 = 2217 raw
    {2081L, 16L},    // Additional threshold for high battery 3800mV in raw = (3800 × 1752) / 3200 = 2081 raw
    {2273U, INT16_MAX} // Extra high threshold to avoid CV oscillations 4150 * 1752 / 3200 = 2273 raw
};
const int16_t fall_threshold[] = {44, -11}; // -20mV rate in raw ADC units
const trend_entry_t trend_detection[] = {{2,16},
    {2,16},
#if ULP_ADC_HISTORY_SIZE == 4
    {4,22},
#elif ULP_ADC_HISTORY_SIZE == 8
    {6,30}
#endif
};
#else
const threshold_entry_t rise_thresholds_mv[] = {
    {3600U, 150L},   // Below 3600mV equivalent 150mv in raw = (150 × 1752) / 3200 = 82 raw
    {3800U, 70L},    // 3600-3800mV equivalent 70mv in raw = (70 × 1752) / 3200 = 38 raw
    {4000U, 50L},    // 3800-4000mV equivalent 50mv in raw = (50 × 1752) / 3200 = 27 raw
    {UINT32_MAX, 40L},     // Above 4000mV equivalent 40mv in raw = (40 × 1752) / 3200 = 22 raw
    {4050U, INT16_MAX},    // Additional threshold for high battery 4050mV in raw = (4050 × 1752) / 3200 = 2217 raw
    {3800U, 20L}     // Additional threshold for high battery 3800mV in raw = (3800 × 1752) / 3200 = 2081 raw
    {4150U, INT16_MAX} // Extra high threshold to avoid CV oscillations
};
const int16_t fall_threshold[] = {80, -20}; // -20mV rate in raw ADC units
const trend_entry_t trend_detection_mv[] = {
    {2,30},
    {6,55}
};
#endif

static bool detect_charge_start(uint32_t raw_adc, adc_analysis_t analysis) {
    if (analysis.trend != TREND_RISING || analysis.rate_of_change <= 0) {
        return false;
    }
    
    // Define thresholds directly in raw ADC units
    uint32_t rise_threshold;

    if (raw_adc < rise_thresh[0].threshold) rise_threshold = rise_thresh[0].rate_of_change;      // Below 3600mV equivalent
    else if (raw_adc < rise_thresh[1].threshold) rise_threshold = rise_thresh[1].rate_of_change;  // 3600-3800mV equivalent
    else if (raw_adc < rise_thresh[2].threshold) rise_threshold = rise_thresh[2].rate_of_change;  // 3800-4000mV equivalent
    else rise_threshold = rise_thresh[3].rate_of_change;                     // Above 4000mV equivalent

    bool significant_rise = (analysis.rate_of_change >= rise_threshold);
    // bool voltage_above_normal = (raw_adc > rise_thresh[4].threshold);  // 4050mV equivalent

    if (raw_adc >= rise_thresh[5].threshold) {  // 3800mV equivalent
        // return significant_rise || (analysis.rate_of_change >= rise_thresh[5].rate_of_change && voltage_above_normal);
        // At high voltage, INCREASE threshold to avoid CV charging oscillations
        // Require either: very strong rise OR moderate rise + very high voltage
        bool strong_rise = (analysis.rate_of_change >= rise_thresh[0].rate_of_change);  // 82 raw = 150mV
        bool extremely_high_voltage = (raw_adc > rise_thresh[6].threshold);  // 4150mV = clearly charging
        return strong_rise || (significant_rise && extremely_high_voltage);
    } else {
        return significant_rise;
    }
}

static bool detect_charge_stop(uint32_t raw_adc, adc_analysis_t analysis, uint32_t peak_raw_adc) {
    // Calculate drop from charging peak (in raw units)
    uint32_t drop_from_peak = peak_raw_adc - raw_adc;
    
    // Convert 80mV drop to raw ADC units
    // 80mV in raw = (80 × 1752) / 3200 = 44 raw
    bool significant_drop = (drop_from_peak >= fall_threshold[0]); // 80mV drop equivalent
    
    // Convert -20mV rate to raw ADC units  
    // -20mV in raw = (20 × 1752) / 3200 = 11 raw (use absolute value)
    bool negative_roc = (analysis.rate_of_change < -fall_threshold[1]);

    bool falling_trend = (analysis.trend == TREND_FALLING);
    
    return significant_drop && falling_trend && negative_roc;
}

void adc_sync_initial_charging_state(bool charging) {
    if (!adc_initial_sync_done) {
        adc_initial_charging_state = charging;
        adc_charging_is_on = charging;  // Set internal charging flag
        ILOG(TAG, "ADC: Initial charging state synced to %d", charging);
    }
}

/**
 * Robust charge detection for real-world scenarios
 * - Adaptive thresholds based on battery level
 * - Multi-factor confirmation
 * - Handles small voltage changes
 */
static adc_battery_state_t get_battery_state(void) {
    static bool is_charging = false;
    static uint32_t last_charge_change_ms = 0;
    // static uint32_t charge_start_voltage = 0;
    static uint32_t charge_peak_voltage = 0;
    const uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    const uint32_t DEBOUNCE_MS = 3000;
    
    /* Use cached snapshot and calibrated value */
    uint32_t voltage_mv = adc_get_cached_batt_mv();
    adc_snapshot_t snap = {0};
    bool have_snap = adc_get_cached_snapshot(&snap);
    uint32_t voltage_mv_raw = have_snap ? (snap.last_sample & 0xFFF) : 0;
    uint8_t available = have_snap ? (uint8_t)snap.valid_count : 0;
    uint32_t history_avg_raw = have_snap ? snap.history_avg : 0;
    uint32_t mad_raw = (have_snap && SNAPSHOT_HAS_MAD(&snap)) ? snap.mad : 0;
    uint32_t current_mv_calibrated = adc_get_cached_batt_mv();
    adc_analysis_t analysis = analyze_adc_readings(voltage_mv_raw, available);
    // ONE-TIME INIT: Set initial state from ULP detection
    if (!adc_initial_sync_done) {
        is_charging = adc_initial_charging_state;
        if (is_charging) {
            ILOG(TAG, "ADC: Using initial charging state from ULP");
        }
        adc_initial_sync_done = true;
    }

    FUNC_ENTRY_ARGS(TAG, "voltage:%lumV, raw: %lu, charging:%d, trend:%d, roc:%ld",
           voltage_mv, voltage_mv_raw, is_charging, analysis.trend, analysis.rate_of_change);
    
    debug_ulp_status();

    // 1. SAFETY FIRST: Critical low always triggers
    if (voltage_mv < BATTERY_CRITICAL_LOW_MV) {
        return ADC_BATTERY_CRITICAL_LOW;
    }
    
    // 2. CHARGE DETECTION (only when debounce period passed)
    if ((current_ms - last_charge_change_ms) >= DEBOUNCE_MS) {
        
        // --- CHARGING STARTED DETECTION ---
        if (!is_charging) {
            bool charge_start_detected = detect_charge_start(voltage_mv_raw, analysis);
            /* Also consider adaptive dynamic threshold (analysis.dyn_threshold).
             * Rate-of-change is in raw units; require consecutive confirmations to avoid flapping.
             */
            /* Use a slightly scaled dyn threshold (75%) to detect earlier while
             * still basing the value on measured noise. This helps avoid missing
             * short but real charge events when MAD is slightly inflated.
             */
            uint32_t scaled_dyn_thr = (analysis.dyn_threshold * 3) / 4; // 75%
            bool dyn_rise = (analysis.rate_of_change >= (int32_t)scaled_dyn_thr);

            if (charge_start_detected || dyn_rise) {
                adc_consec_up++;
                adc_consec_down = 0;
                DLOG(TAG, "dyn_start candidate: roc=%ld dyn=%lu scaled=%lu mad=%lu consec_up=%u",
                     analysis.rate_of_change, (unsigned long)analysis.dyn_threshold, (unsigned long)scaled_dyn_thr,
                     (unsigned long)analysis.noise_mad, adc_consec_up);
            } else {
                if (adc_consec_up != 0) DLOG(TAG, "reset consec_up (was %u)", adc_consec_up);
                adc_consec_up = 0;
            }

            if (adc_consec_up >= ADC_CONSEC_REQUIRED) {
                is_charging = true;
                adc_consec_up = 0;
                adc_consec_down = 0;
                last_charge_change_ms = current_ms;
                charge_peak_voltage = voltage_mv_raw;
                ILOG(TAG, "CHARGING STARTED (confirmed): %lu mV (+%ld), dyn_thr=%lu, mad=%lu",
                     voltage_mv_raw, analysis.rate_of_change, (unsigned long)analysis.dyn_threshold, (unsigned long)analysis.noise_mad);
                return ADC_BATTERY_CHARGING_STARTED;
            }
        }
        
        // --- CHARGING STOPPED DETECTION ---
        if (is_charging) {
            bool charge_stop_detected = detect_charge_stop(voltage_mv_raw, analysis, charge_peak_voltage);
            /* Consider dynamic threshold for falling edge as well. Use scaled version (75%). */
            uint32_t scaled_dyn_thr_f = (analysis.dyn_threshold * 3) / 4; // 75%
            bool dyn_fall = (analysis.rate_of_change <= -(int32_t)scaled_dyn_thr_f);

            if (charge_stop_detected || dyn_fall) {
                adc_consec_down++;
                adc_consec_up = 0;
                DLOG(TAG, "dyn_stop candidate: roc=%ld dyn=%lu scaled=%lu mad=%lu consec_down=%u",
                     analysis.rate_of_change, (unsigned long)analysis.dyn_threshold, (unsigned long)scaled_dyn_thr_f,
                     (unsigned long)analysis.noise_mad, adc_consec_down);
            } else {
                if (adc_consec_down != 0) DLOG(TAG, "reset consec_down (was %u)", adc_consec_down);
                adc_consec_down = 0;
            }

            if (adc_consec_down >= ADC_CONSEC_REQUIRED) {
                is_charging = false;
                adc_consec_down = 0;
                adc_consec_up = 0;
                last_charge_change_ms = current_ms;
                charge_peak_voltage = 0;
                ILOG(TAG, "CHARGING STOPPED (confirmed): %lu mV", voltage_mv_raw);
                return ADC_BATTERY_CHARGING_STOPPED;
            }

            // Update peak voltage during charging
            if (voltage_mv_raw > charge_peak_voltage) {
                charge_peak_voltage = voltage_mv_raw;
            }
        }
    }
    
    // 3. STATE REPORTING
    if (is_charging) {
        // Check if charging has stabilized (been active for more than 30 seconds)
        const uint32_t CHARGE_STABILIZE_MS = 30000; // 30 seconds
        if ((current_ms - last_charge_change_ms) >= CHARGE_STABILIZE_MS) {
            return ADC_BATTERY_CHARGE_STABILIZED;
        } else {
            return ADC_BATTERY_CHARGING_STARTED;
        }
    } else {
        // Battery level reporting
        if (voltage_mv < BATTERY_LOW_MV) {
            return ADC_BATTERY_LOW;
        } else if (voltage_mv >= BATTERY_HIGH_MV) {
            /* If voltage is in HIGH range but recent trend indicates a clear rising
             * rate (based on adaptive dyn threshold), prefer reporting CHARGING_STARTED
             * so we don't emit Normal->High transitions when the charger was just
             * connected. This helps UI/logic which expects charge events.
             */
            uint32_t scaled_dyn_thr = (analysis.dyn_threshold * 3) / 4; // 75%
            if (!is_charging && analysis.rate_of_change >= (int32_t)scaled_dyn_thr) {
                // Treat as charging started
                is_charging = true;
                last_charge_change_ms = current_ms;
                ILOG(TAG, "Voltage in HIGH range and rising -> treat as CHARGING_STARTED: %lu mV (roc=%ld dyn=%lu)",
                     voltage_mv_raw, analysis.rate_of_change, (unsigned long)analysis.dyn_threshold);
                return ADC_BATTERY_CHARGING_STARTED;
            }
            return ADC_BATTERY_HIGH;
        } else {
            return ADC_BATTERY_NORMAL;
        }
    }
}

/**
 * Simple analyzer focused on charge detection
 */
static adc_analysis_t analyze_adc_readings(uint32_t voltage_mv, uint8_t available) {
    adc_analysis_t result = {0};
    result.filtered_reading = voltage_mv;
    /* Unified sample acquisition (ULP or recent buffer) */
    uint32_t current = 0, prev1 = 0, prev2 = 0;
    ulp_get_three_samples(&current, &prev1, &prev2);
    DLOG(TAG, "ADC readings: current=%lu, prev1=%lu, prev2=%lu", current, prev1, prev2);

    if (available >= trend_detection[1].readings) {
        
        // Use medium-term rate of change (more stable)
        result.rate_of_change = (int32_t)current - (int32_t)prev2;
        
        // FIXED: More conservative trend detection
        if (result.rate_of_change > trend_detection[1].threshold) {
            result.trend = TREND_RISING;
        } else if (result.rate_of_change < -(trend_detection[1].threshold)) {
            result.trend = TREND_FALLING;
        } else {
            result.trend = TREND_STABLE;
        }

    } else if (available >= trend_detection[0].readings) {
        // Basic for small buffers
        result.rate_of_change = (int32_t)current - (int32_t)prev1;
        result.trend = (result.rate_of_change > trend_detection[0].threshold) ? TREND_RISING :
                      (result.rate_of_change < -(trend_detection[0].threshold)) ? TREND_FALLING : TREND_STABLE;
    } else {
        result.trend = TREND_STABLE;
    }

    /* Compute noise MAD using unified helper that may use ULP snapshot */
    result.noise_mad = compute_history_mad(available);
    uint32_t dyn_thr = ADC_RAPID_CHANGE_THRESHOLD;
    uint32_t cand = result.noise_mad * ADC_NOISE_MULTIPLIER;
    if (cand > dyn_thr) dyn_thr = cand;
    result.dyn_threshold = dyn_thr;
    
    return result;
}

/**
 * Handle battery state machine for regular ADC mode (when ULP is disabled)
 * Called from adc_update() to process new voltage readings
 */
static void handle_adc_battery_state(void) {
    uint32_t voltage_mv = adc_get_cached_batt_mv();
    adc_battery_state_t new_state = get_battery_state();
// #if defined(CONFIG_LOGGER_ADC_MODE_ULP)
//     uint32_t voltage_mv = calibrate_adc_raw(ULP_GET_U32(ulp_last_result));
// #else
//     uint32_t voltage_mv = get_recent_reading(0); // Get voltage from buffer
// #endif
    FUNC_ENTRY_ARGSD(TAG, "new_state: %s (%d), last_state: %s (%d)", 
                    adc_battery_states_str(new_state), new_state, adc_battery_states_str(last_adc_battery_state), last_adc_battery_state);

    // Only post events on state changes
    if (new_state != last_adc_battery_state) {
        // const char* state_names[] = {"NORMAL", "LOW", "HIGH", "CHARGING_STARTED", "CHARGING_STOPPED", "CRITICAL_LOW"};
        
        // Check if we should suppress state changes during transitions
        if (s_adc_events_suppressed) {
            // Always allow critical low battery state changes - safety first
            if (new_state != ADC_BATTERY_CRITICAL_LOW) {
                DLOG(TAG, "State change suppressed during transition: %s -> %s (%lu mV)", 
                     adc_battery_states_str(last_adc_battery_state), adc_battery_states_str(new_state), voltage_mv);
                return;  // Suppress the state change entirely
            }
        }
        
        DLOG(TAG, "State change: %s -> %s (%lu mV)", 
             adc_battery_states_str(last_adc_battery_state), adc_battery_states_str(new_state), voltage_mv);
        
        // Priority-based logging
        if (new_state == ADC_BATTERY_CHARGING_STARTED) {
            ILOG(TAG, "PRIORITY 1: Charging started event");
        } else if (new_state == ADC_BATTERY_CRITICAL_LOW) {
            ELOG(TAG, "PRIORITY 2: Critical low battery!");
        } else if (new_state == ADC_BATTERY_CHARGING_STOPPED) {
            ILOG(TAG, "PRIORITY 3: Charging stopped event");
        }
        
        post_battery_state_event(new_state, "ADC");
        
        if (new_state == ADC_BATTERY_CHARGING_STARTED || 
            new_state == ADC_BATTERY_CHARGING_STOPPED) {
            force_instant_voltage = true;
        }
        

        last_adc_battery_state = new_state;
        
        // Sync charging state to ULP
// #if defined(CONFIG_ULP_COPROC_ENABLED)
//         bool charging = (new_state == ADC_BATTERY_CHARGING_STARTED);
//         ULP_SET_U32(ulp_charging_active, charging ? 1 : 0);
// #endif
    }
}

#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
/**
 * Diagnostic function to help identify ADC pin conflicts on LilyGO boards
 * Call this during initialization to check for potential issues
 */
void diagnose_adc_pin_conflicts(void) {
    ILOG(TAG, "=== ADC Pin Conflict Diagnostic (LilyGO Board) ===");
    ILOG(TAG, "ADC Channel: %d", CONFIG_ADC_CHANNEL);
    
    // Take multiple quick readings to check for stability
    float readings[5];
    bool stable = true;
    
    for (int i = 0; i < 5; i++) {
        readings[i] = adc_get_cached_batt_volt();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Check for excessive variation (sign of interference)
    float min_v = readings[0], max_v = readings[0];
    for (int i = 1; i < 5; i++) {
        if (readings[i] < min_v) min_v = readings[i];
        if (readings[i] > max_v) max_v = readings[i];
    }
    
    float variation = max_v - min_v;
    if (variation > 0.5f) {  // > 500mV variation suggests interference
        stable = false;
        WLOG(TAG, "High voltage variation detected: %.3fV - possible pin conflict", variation);
    }
    
    ILOG(TAG, "Voltage readings: %.3f, %.3f, %.3f, %.3f, %.3f", 
         readings[0], readings[1], readings[2], readings[3], readings[4]);
    ILOG(TAG, "Variation: %.3fV, Stable: %s", variation, stable ? "YES" : "NO");
    
    if (!stable) {
        WLOG(TAG, "ADC pin may be shared with other peripherals:");
        WLOG(TAG, "- Check if SD card, sensors, or display power management use same pin");
        WLOG(TAG, "- Consider using ULP readings which may be more stable");
        WLOG(TAG, "- Verify pin configuration in board documentation");
    } else {
        ILOG(TAG, "ADC readings appear stable");
    }
    
    ILOG(TAG, "=== End ADC Diagnostic ===");
}
#endif

/* Low battery timer callback - handles final shutdown trigger */
static void adc_low_bat_timer_cb(void *arg) {
    if (bat_safe_lock(50)) {
        WLOG(TAG, "Low battery timer expired - triggering shutdown callback");
        if (low_battery_callback) {
            low_battery_callback();
        }
        bat_safe_unlock();
    }
}

uint8_t adc_on_ac() {
    return last_adc_battery_state == ADC_BATTERY_CHARGING_STARTED ? 1 : 0;
}

// uint32_t calc_bat_perc(float adc) {
//     ILOG(TAG, "[%s] %0.4f", __func__, adc);
//     uint32_t adck = adc * 1000;
//     uint32_t bat_perc = VOLTAGE_PERC(adck);
// #if (C_LOG_LEVEL < 1)
//     DLOG(TAG, "[%s] voltage: %f converted: %lu mV perc: %lu coef: %lu", __func__, adc, adck, bat_perc, VOLTAGE_PERC_COEF(adck));
// #endif
//     if (bat_perc < 0)
//         bat_perc = 0;
//     else if (bat_perc > 100)
//         bat_perc = 100;
//     return bat_perc;
// }

static const char * cali_mode = "";

uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten) 
{
    FUNC_ENTRY(TAG);
    esp_err_t ret = ESP_FAIL;
    if (adc_ctx.cali_handle) return true;

#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    cali_mode = "Curve Fitting";
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = _ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_ctx.cali_handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    cali_mode = "Line Fitting";
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = _ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_ctx.cali_handle);
#endif
    ILOG(TAG,"[%s] calibration scheme version is %s", __func__, cali_mode);
    if (ret == ESP_OK) {
        ILOG(TAG,"[%s] Calibration Success", __func__);
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        WLOG(TAG, "[%s] eFuse not burnt, skip software calibration", __func__);
    } else {
        ELOG(TAG, "[%s] Invalid arg or no memory", __func__);
    }
    return !ret && adc_ctx.cali_handle && (adc_ctx.do_calibration = true);
}

void adc_calibration_deinit(void) {
#if (C_LOG_LEVEL > LOG_DEBUG_NUM)
    FUNC_ENTRY(TAG);
#else
    FUNC_ENTRY_ARGSD(TAG, " deregister %s calibration scheme", cali_mode);
#endif
    if(!adc_ctx.do_calibration) return;
    adc_ctx.do_calibration = false;
    if(!adc_ctx.cali_handle) return;
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_curve_fitting(adc_ctx.cali_handle)) {
        ELOG(TAG, "[%s] Failed to delete curve fitting scheme", __func__);
    }
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_line_fitting(adc_ctx.cali_handle)) {
        ELOG(TAG, "[%s] Failed to delete line fitting scheme", __func__);
    }
#endif
    adc_ctx.cali_handle = NULL;
}

#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
/*
* Read raw ADC value and apply calibration if enabled
*/
// static uint32_t adc_read_raw() {
//     // esp_err_t err = 0;
// #if defined(CONFIG_LOGGER_ADC_MODE_ULP)
//     // ULP mode: Use last ULP reading
//     uint32_t v = ULP_GET_U32(ulp_last_result);
// #elif defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
//     int v = 0;
//     if(!adc_ctx.adc1_handle || adc_oneshot_read(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &v)) {
//         ELOG(TAG, "[%s] Failed to read ADC %d", __func__, _ADC_CHANNEL_0);
//         return 0;
//     }
//     adc_ctx.adc_raw = v;
// #endif
//     adc_ctx.adc_voltage = calibrate_adc_raw((uint32_t)v);
//     // TLOG(TAG, "[%s] ADC%d channel[%d]: raw: %lu, calibrated: %lu", __func__, _ADC_UNIT_0 + 1, _ADC_CHANNEL_0, adc_ctx.adc_raw, adc_ctx.adc_voltage);
//     return adc_ctx.adc_voltage;
// }
#endif

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

static uint32_t adc_read_count(uint16_t count, uint16_t delay) {
    FUNC_ENTRY(TAG);

    if (count == 0) return 0;
    if (count > 16) count = 16;
    
    uint32_t sum = 0;
    uint32_t min_val = UINT32_MAX;
    uint32_t max_val = 0;

    // Single pass: calculate sum and find min/max for outlier rejection
    for (uint16_t i = 0; i < count; i++) {
        uint32_t reading = adc_read_raw();
        sum += reading;
        
        if (reading < min_val) min_val = reading;
        if (reading > max_val) max_val = reading;
        
        if (delay) vTaskDelay(pdMS_TO_TICKS(delay));
    }

    // Simple outlier rejection: remove min and max if we have enough samples
    if (count >= 5) {
        sum = sum - min_val - max_val;
        count -= 2;
    }
    return (sum / count) * 100;
}
#endif

#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
static uint8_t result_avg_efficient() {
    uint8_t index = get_current_buffer_index();
    uint8_t prev_index = get_previous_buffer_index();
    if(adc_ctx.result_index >= RESULT_SIZE) adc_ctx.running_sum -= (adc_ctx.result[prev_index]);
    adc_ctx.running_sum += (adc_ctx.result[index]);
    if(adc_ctx.result_index >= RESULT_SIZE) {
        if(index == 0) {
            if(adc_ctx.m_avg[1]) adc_ctx.m_avg[2] = adc_ctx.m_avg[1]; // 2. RESULT_SIZE avg set
            if(adc_ctx.m_avg[0]) adc_ctx.m_avg[1] = adc_ctx.m_avg[0]; // 1. RESULT_SIZE avg set
            adc_ctx.m_avg[0] = adc_ctx.running_avg; // previous RESULT_SIZE avg set
            TLOG(TAG, "[%s] new set index 0, avg updated", __func__);
        }
        adc_ctx.running_avg = adc_ctx.running_sum / RESULT_SIZE;
        TLOG(TAG,"[%s] prev avg: {%lu, %lu, %lu}, avg: %lu, index: %hhu", __func__, adc_ctx.m_avg[2], adc_ctx.m_avg[1], adc_ctx.m_avg[0], adc_ctx.running_avg, index);
    }
    return (adc_ctx.m_avg[2] && adc_ctx.m_avg[0] > adc_ctx.m_avg[2]) ? 1 : 0;
}
#endif

#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
// ADC update function called periodically from timer to read voltage and handle state
static void adc_update(void*arg) {
    FUNC_ENTRYD(TAG);
    // Take 11 readings with 5ms delay between each for better stability on LilyGO T5 charging circuits
    // Increased sample count and delay to handle rapid voltage fluctuations during charging
    uint32_t cal_reading = 0;
    adc_snapshot_t tmp_snap = {0};
    if(adc_lock(100)) {
        /* Populate module-level cache so other functions can read a single
        * consistent snapshot and the calibrated battery mV value. */
        if (adc_snapshot_take(&tmp_snap, true, 3)) {
            s_cached_snapshot = tmp_snap;
            s_cached_snapshot_valid = true;
        } else {
            s_cached_snapshot_valid = false;
        }
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
        cal_reading = calibrate_adc_raw(tmp_snap.last_sample);
#else
        cal_reading = adc_read_count(11, 5);
        add_adc_reading(cal_reading);
#endif
        /* Reading is in millivolts already (converted above) */
        s_cached_batt_mv = VOLTAGE_CONV_ADC_TO_MV_UL(cal_reading);
        adc_unlock();
    }
    FUNC_ENTRY_ARGSD(TAG, "got reading:%lu, converted_to_mv:%lu", cal_reading, s_cached_batt_mv);

    handle_adc_battery_state();

    // Integrated low battery monitoring and RTC voltage update - runs with every ADC update
    if (bat_safe_lock(10)) {
        float current_voltage = VOLTAGE_CONV_MV_TO_V(s_cached_batt_mv);  // Convert millivolts to volts
        // printf ("Voltage: %.3f V\n", current_voltage);
        // Post voltage update event for main.c to handle RTC context updates  
        esp_event_post(ADC_EVENT, ADC_EVENT_UPDATE, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(50));
        
        // Low battery monitoring - trigger shutdown callback when battery is critically low
        static uint32_t low_bat_start_time = 0;
        
        if (s_cached_batt_mv < BATTERY_CRITICAL_LOW_MV) {
            uint32_t now = get_millis();
            
            if (low_bat_start_time == 0) {
                low_bat_start_time = now;
                ELOG(TAG, "Low battery detected: %lu mV < %lu mV - starting countdown", 
                     s_cached_batt_mv, BATTERY_CRITICAL_LOW_MV);
            } else if ((now - low_bat_start_time) > LOW_BAT_SEQUENCE_TIME_MS) {
                ELOG(TAG, "Battery critically low for %d seconds - triggering shutdown", 
                     (int)FROM_K_UL(LOW_BAT_SEQUENCE_TIME_MS));
                if (low_battery_callback) {
                    low_battery_callback();
                }
                low_bat_start_time = 0; // Reset to avoid repeated calls
            }
        } else {
            // Battery voltage is OK - reset countdown
            low_bat_start_time = 0;
        }

        bat_safe_unlock();
    }
#if defined(AC_DETECTABLE)
    uint8_t on_ac = 0;
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    on_ac = gpio_get_level(GPIO_NUM_15);
#else
    on_ac = result_avg_efficient();
#endif
    if (on_ac != adc_ctx.on_ac) {
        adc_ctx.on_ac = on_ac;
        adc_charging_is_on = on_ac;  // Update internal charging flag
        adc_lcd_charge_notification = true;  // Notify LCD of charge event
        esp_event_post(ADC_EVENT, on_ac ? ADC_EVENT_CHARGE_STARTED : ADC_EVENT_CHARGE_STOPPED, &adc_ctx.on_ac, sizeof(adc_ctx.on_ac), portMAX_DELAY);
    }
#endif
}
#endif

#if defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_ctx.adc_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void adc_task(void * arg) {
    FUNC_ENTRY(TAG);
    esp_err_t ret;
    uint8_t count = 0;
    while (adc_ctx.task_is_running && adc_ctx.adc1_handle) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ret = adc_continuous_read(adc_ctx.adc1_handle, adc_ctx.result, READ_LEN, &adc_ctx.ret_num, 0);
        if (ret == ESP_OK) {
            for (int i = 0; i < adc_ctx.ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_ctx.result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);
                if (chan_num == SOC_ADC_CHANNEL_NUM(_ADC_CHANNEL_0)) adc_ctx.adc_raw = data;
            }
        }
        if (count > 10) count = 0;
        if (count == 0) {
            DLOG(TAG,"[%s] adc_raw: %lu", __func__, adc_ctx.adc_raw);
        }
        ++count;
        delay_ms(10);
    }
    vTaskDelete(NULL);
}

#endif

esp_err_t adc_init(void) {
    FUNC_ENTRY(TAG);
    if(adc_ctx.adc_initialized) return ESP_OK; // Already initialized
    esp_err_t ret = 0;

    // Setup mutex for thread-safe ADC access
    if(adc_ctx.xMutex == NULL) adc_ctx.xMutex = xSemaphoreCreateMutex();
    if(adc_ctx.xMutex == NULL) {
        ELOG(TAG, "[%s] Failed to create mutex", __func__);
        return ESP_FAIL;
    }

    adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN);
    
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    /* ULP Primary Mode: Use ULP's raw ADC with manual voltage conversion */
    /* Note: ULP configures GPIO in RTC mode, which disconnects it from digital ADC */
    /* Therefore, we skip ADC calibration and rely on VOLTAGE_CONV macro instead */
    ILOG(TAG, "ULP as primary ADC source - using manual voltage conversion");
    resume_ulp_program();
#elif defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = _ADC_UNIT_0,
    };
    if(adc_oneshot_new_unit(&init_config1, &adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to create ADC unit", __func__);
        return ESP_FAIL;
    }
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = _ADC_BITWIDTH,
        .atten = _ADC_ATTEN,
    };
    if(adc_oneshot_config_channel(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &adc_config)) {
        ELOG(TAG, "[%s] Failed to config ADC channel", __func__);
        return ESP_FAIL;
    }
#endif

    // Initialize periodic ADC tasks for regular readings
#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_update(0);
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &adc_update,
        .name = "periodic_adc",
        .arg = NULL
    };
    if(esp_timer_create(&periodic_timer_args, &adc_ctx.adc_periodic_timer)){
        ELOG(TAG, "[%s] Failed to create periodic timer", __func__);
        return ESP_FAIL;
    }
    if(esp_timer_start_periodic(adc_ctx.adc_periodic_timer, MS_TO_US(ADC_UPDATE_INTERVAL_MS))) {
        ELOG(TAG, "[%s] Failed to start periodic timer", __func__);
        return ESP_FAIL;
    }
#else
    memset(&adc_ctx.result[0], 0xcc, READ_LEN);
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 256,
        .conv_frame_size = READ_LEN,
    };
    if(adc_continuous_new_handle(&adc_config, &adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to create ADC unit", __func__);
        return ESP_FAIL;
    }
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
        .pattern_num = 1,
    };

    adc_digi_pattern_config_t adc_patterns[1] = {0};
    dig_cfg.pattern_num = 0;
    adc_patterns[0].atten = _ADC_ATTEN;
    adc_patterns[0].channel = _ADC_CHANNEL_0;
    adc_patterns[0].unit = _ADC_UNIT_0;
    adc_patterns[0].bit_width = _ADC_BITWIDTH;
    DLOG(TAG, "adc_patterns[0].atten is 0x%"PRIx8"", adc_patterns[0].atten);
    DLOG(TAG, "adc_patterns[0].channel is 0x%"PRIx8"", adc_patterns[0].channel);
    DLOG(TAG, "adc_patterns[0].unit is 0x%"PRIx8"", adc_patterns[0].unit);
    dig_cfg.adc_pattern = adc_patterns;
    if(adc_continuous_config(adc_ctx.adc1_handle, &dig_cfg)){
        ELOG(TAG, "[%s] Failed to config ADC continuous", __func__);
        return ESP_FAIL;
    }
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    xTaskCreatePinnedToCore(adc_task, "ADC Task", (8*256), NULL, 0, &adc_ctx.adc_task_handle, 1);
    if(adc_continuous_register_event_callbacks(adc_ctx.adc1_handle, &cbs, NULL)) {
        ELOG(TAG, "[%s] Failed to register event callbacks", __func__);
        return ESP_FAIL;
    }
    if(adc_continuous_start(adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to start ADC continuous", __func__);
        return ESP_FAIL;
    }
    delay_ms(200);
#endif
    // Initialize battery safety mutex
    if (!adc_ctx.batMutex) {
        adc_ctx.batMutex = xSemaphoreCreateMutex();
        if (!adc_ctx.batMutex) {
            ELOG(TAG, "Failed to create battery safety mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    adc_ctx.adc_initialized = true;
    return ret;
}

esp_err_t adc_deinit() {
    FUNC_ENTRY(TAG);
    if(!adc_ctx.adc_initialized) return ESP_OK; // Not initialized
    adc_ctx.adc_initialized = false;
    esp_err_t err = 0;
    if(adc_lock(-1)) {
        adc_unlock();
    }
#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    if (adc_ctx.adc_periodic_timer) {
        esp_timer_stop(adc_ctx.adc_periodic_timer);
        esp_timer_delete(adc_ctx.adc_periodic_timer);
        adc_ctx.adc_periodic_timer = NULL;
    }
#else
    adc_ctx.task_is_running = 0;
    if(adc_ctx.adc1_handle) {
        xTaskNotifyGive(adc_ctx.adc_task_handle);
        adc_continuous_stop(adc_ctx.adc1_handle);
        adc_continuous_deinit(adc_ctx.adc1_handle);
    }
#endif
    if(adc_ctx.xMutex != NULL){
        vSemaphoreDelete(adc_ctx.xMutex);
        adc_ctx.xMutex = NULL;
    }
    
    // Cleanup low battery timer
    if (adc_ctx.low_bat_timer) {
        esp_timer_stop(adc_ctx.low_bat_timer);
        esp_timer_delete(adc_ctx.low_bat_timer);
        adc_ctx.low_bat_timer = NULL;
    }
    
    // Cleanup battery safety mutex
    if(adc_ctx.batMutex != NULL){
        vSemaphoreDelete(adc_ctx.batMutex);
        adc_ctx.batMutex = NULL;
    }
    low_battery_callback = NULL;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    if (adc_ctx.adc1_handle) {
        adc_oneshot_del_unit(adc_ctx.adc1_handle);
        adc_ctx.adc1_handle = NULL;
    }
#endif
    adc_calibration_deinit();
    return err;
}

float adc_get_cached_batt_volt(void) {
    FUNC_ENTRY(TAG);
    float voltage;
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)    
    // Validate reading against board-specific thresholds
    voltage = validate_and_clamp_voltage_mv(s_cached_batt_mv);
#elif defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    // Always use the most recent reading, no smoothing
    if (adc_lock(100)) {
        voltage = get_recent_voltage_reading();
        adc_unlock();
    } else {
        voltage = get_recent_voltage_reading();
    }
    force_instant_voltage = false; // Clear flag after use (if set)
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    voltage = VOLTAGE_CONV_V(adc_ctx.adc_raw);
#endif

    ILOG(TAG, "[%s] adc_raw: %lu, cali_v: %lu, volt: %f", __func__, ULP_GET_U32(ulp_last_result), s_cached_batt_mv, voltage);
    return voltage;
}

/* Battery monitoring API - thread-safe access to battery data */
bool adc_check_battery_level(void) {
    if (!adc_ctx.adc_initialized) {
        return true; // Default to safe if not initialized
    }
    return (adc_get_cached_batt_mv() >= BATTERY_CRITICAL_LOW_MV);
}

void adc_set_low_battery_callback(void (*callback)(void)) {
    if (bat_safe_lock(50)) {
        low_battery_callback = callback;
        bat_safe_unlock();
    } else {
        // Fallback assignment without mutex
        low_battery_callback = callback;
    }
}

/**
 * Get battery voltage optimized for display updates
 * Parameters: raw_adc_value (0 if not available), fallback_voltage, output pointer
 * Uses reference parameter to avoid return value overhead - more efficient for frequent calls
 */
void get_battery_voltage_for_display(float *voltage_out) {
#if defined(CONFIG_LOGGER_ADC_ENABLED)
    if (!voltage_out) return; // Safety check
    if (s_cached_batt_mv > 0) {
        *voltage_out = adc_mv_to_voltage(s_cached_batt_mv);
    } else {
        *voltage_out = adc_mv_to_voltage(FALLBACK_VOLTAGE_MV);
    }
#else
    if (voltage_out) *voltage_out = 3.6f; // Default fallback when ADC disabled
#endif
}

uint8_t calc_bat_perc_v(float adc) {
    FUNC_ENTRY_ARGS(TAG, "%.04f", adc);
    uint32_t kadc = adc * 10000, sv, step, v, v1;
    uint8_t i=0, ret = 0, perc=0;
    if(kadc<=v_graph_lipo[0]) {
        ret = 0;
    }
    else if(kadc<=v_graph_lipo[V_GRAPH_LIPO_LEN-1]){
        for(;i<V_GRAPH_LIPO_LEN;++i, perc+=5) { // 0-100%
            v=v_graph_lipo[i]; // 32700
            v1=v_graph_lipo[i+1]; // 36100
            if(kadc == v) { // 32700
                ret = perc;
                goto done;
            }
            else if(kadc == v1) { // 36100
                ret = perc+5;
                goto done;
            }
            else if(kadc<v1) { // between 32700 and 36100
                step = (v1 - v) / 5; // divide by 1% for steps
                ++perc;
                for(sv=v+step;sv<=v1;sv+=step,++perc) {
                    if(kadc<=sv) {
                        ret = perc;
                        goto done;
                    }
                }
            }
        }
    } else ret = 100;
    done:
    DLOG(TAG,"[%s] voltage: %f converted: %lu mV perc: %hhu", __func__, adc, kadc, ret);
    return ret;
}

/* ADC event suppression functions - prevent false events during system transitions */
void adc_suppress_events(const char* reason) {
    s_adc_events_suppressed = true;
    s_adc_suppression_start_time = FROM_K_UL(esp_timer_get_time());  // Convert to ms
    ILOG(TAG, "[%s] ADC events suppressed: %s", __func__, reason);
}

void adc_resume_events(const char* reason) {
    s_adc_events_suppressed = false;
    s_adc_suppression_start_time = 0;
    ILOG(TAG, "[%s] ADC events resumed: %s", __func__, reason);
}

bool adc_should_suppress_event(int32_t event_id) {
    if (!s_adc_events_suppressed) {
        return false;
    }
    
    // Auto-resume after timeout to prevent permanent suppression
    int64_t current_time = FROM_K_UL(esp_timer_get_time());
    if (current_time - s_adc_suppression_start_time > ADC_SUPPRESSION_TIMEOUT_MS) {
        WLOG(TAG, "[%s] ADC suppression timeout, auto-resuming", __func__);
        adc_resume_events("timeout");
        return false;
    }
    
    // Suppress charge-related events during transitions (state changes are prevented at source)
    if (event_id == ADC_EVENT_CHARGE_STARTED || event_id == ADC_EVENT_CHARGE_STOPPED) {
        DLOG(TAG, "[%s] Suppressing ADC charge event during transition: %s", __func__, adc_event_strings(event_id));
        return true;
    }
    
    // Allow critical battery events to pass through
    if (event_id == ADC_EVENT_BATTERY_CRITICAL) {
        WLOG(TAG, "[%s] Allowing critical battery event despite suppression", __func__);
        return false;
    }
    
    // Suppress other battery state changes during transitions
    return true;
}

/* Charging state management - single source of truth */
bool get_adc_charging_state(void) {
    return adc_charging_is_on;
}

/* LCD charge notification flag - check and clear atomically */
bool adc_check_and_clear_lcd_charge_flag(void) {
    bool was_set = adc_lcd_charge_notification;
    if (was_set) {
        adc_lcd_charge_notification = false;
        DLOG(TAG, "[%s] LCD charge notification flag cleared", __func__);
    }
    return was_set;
}

#endif // CONFIG_LOGGER_ADC_ENABLED
