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

#if (C_LOG_LEVEL < 3)
const char * adc_battery_states_str[] = { ADC_BAT_STATES(STRINGIFY) };
const char * adc_ulp_wake_sources_str[] = { ADC_ULP_WAKE_SOURCES(STRINGIFY) };
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

ESP_EVENT_DEFINE_BASE(ADC_EVENT);

/* Battery monitoring integration with main application */
static void (*low_battery_callback)(void) = NULL;
static SemaphoreHandle_t battery_safety_mutex = NULL;

/* Low battery monitoring state - managed by ADC timer */
static esp_timer_handle_t low_bat_timer = NULL;

static float minimum_battery_voltage = 3.25f;  // Default, can be updated
#define LOW_BAT_SEQUENCE_TIME_MS (20 * 1000)  // 20 seconds in milliseconds

/* ADC event suppression during WiFi/GPS transitions */
static bool s_adc_events_suppressed = false;
static int64_t s_adc_suppression_start_time = 0;
#define ADC_SUPPRESSION_TIMEOUT_MS 5000  // 5 seconds max suppression


static const char * _adc_event_strings[] = { ADC_EVENT_LIST(STRINGIFY) };
const char * adc_event_strings(int id) {
    return _adc_event_strings[id];
}

static const char *TAG = "adc";

#define V_GRAPH_LIPO_LEN 21
#define ADJ_LENGTH 24

typedef struct {
    uint32_t result[RESULT_SIZE];
    uint8_t head;      // Next write position
    uint8_t count;
} adc_buffer_t;

typedef struct adc_context_s {
    uint8_t adc_initialized;
    uint8_t on_ac;
    uint32_t adc_raw;
    uint32_t adc_voltage;
    uint8_t do_calibration;
    adc_cali_handle_t adc1_cali_handle;
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    uint32_t running_sum;
    uint32_t running_avg;
    uint32_t m_avg[3];
#endif
    adc_buffer_t adc_buffer;
    SemaphoreHandle_t xMutex;
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_handle_t adc1_handle;
    esp_timer_handle_t adc_periodic_timer;
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_continuous_handle_t adc1_handle;
    TaskHandle_t adc_task_handle;
    uint32_t ret_num;
    uint8_t result[READ_LEN];
    uint8_t task_is_running;
#endif
} adc_context_t;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#define CTX_PART .adc_periodic_timer = NULL,
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#define CTX_PART .adc_task_handle = NULL, \
    .ret_num = 0, \
    .task_is_running = 1,
#endif
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
#define AC_DET_PART .on_ac = 0, \
    .running_sum = 0, \
    .running_avg = 0, \
    .m_avg = {0},
#else
#define AC_DET_PART
#endif
#define ADC_CONTEXT_DEFAULT { \
    .adc_initialized = 0, \
    .on_ac = 0, \
    .adc_raw = 0, \
    .adc_voltage = 0, \
    .do_calibration = 0, \
    .adc1_cali_handle = NULL, \
    .adc1_handle = NULL, \
    .adc_buffer = {{0},0,0}, \
    .xMutex = NULL, \
    AC_DET_PART \
    CTX_PART \
}
static adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;
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

static inline void add_adc_reading(uint32_t value) {
    adc_ctx.adc_buffer.result[adc_ctx.adc_buffer.head] = value;
    adc_ctx.adc_buffer.head = (adc_ctx.adc_buffer.head + 1) & RESULT_MASK;
    adc_ctx.adc_buffer.count += (adc_ctx.adc_buffer.count < RESULT_SIZE);
}

static inline uint32_t get_recent_reading(uint8_t pos) {
    return adc_ctx.adc_buffer.result[(adc_ctx.adc_buffer.head - 1 - pos) & RESULT_MASK];
}

static uint32_t get_recent_average_n(uint8_t num_readings) {
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
    
    return sum / num_readings;
}

static uint32_t get_recent_average(void) {
    return get_recent_average_n(adc_ctx.adc_buffer.count);
}

static uint32_t get_progressive_average(void) {
    uint8_t available = adc_ctx.adc_buffer.count;
    
    if (available == 0) return 0;
    
    // Progressive stages based on buffer fill percentage
    uint8_t fill_percent = (available * 100) / RESULT_SIZE;
    
    if (fill_percent < 25) {
        // 0-25% filled: use current reading or tiny average
        return (available < 2) ? get_recent_reading(0) : get_recent_average_n(2);
    }
    else if (fill_percent < 50) {
        // 25-50% filled: use 25% of buffer size
        return get_recent_average_n(RESULT_SIZE / 4);
    }
    else if (fill_percent < 75) {
        // 50-75% filled: use 50% of buffer size  
        return get_recent_average_n(RESULT_SIZE / 2);
    }
    else if (fill_percent < 90) {
        // 75-90% filled: use 75% of buffer size
        return get_recent_average_n((RESULT_SIZE * 3) / 4);
    }
    else {
        // 90-100% filled: use 90% of buffer size (avoid very oldest readings)
        return get_recent_average_n((RESULT_SIZE * 9) / 10);
    }
}

adc_battery_state_t get_adc_state(void) {
    FUNC_ENTRY_ARGS(TAG," *** %s ***", adc_battery_states_str[last_adc_battery_state]);
    return last_adc_battery_state;
}

/**
 * Common voltage validation function for both ULP and regular ADC
 * Validates voltage readings against board-specific thresholds
 * @param voltage_mv Voltage in millivolts
 * @param source_name Source description for logging ("ULP" or "ADC")
 * @return true if voltage is valid, false if likely pin conflict or unrealistic
 */
bool validate_voltage_reading(uint32_t voltage_mv, const char* source_name) {
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    // T-Display S3: Can show > 4.3V during charging, > 5.5V suggests pin conflict
    if (voltage_mv > 5500) {  // Above realistic USB charging voltage
        WLOG(TAG, "%s voltage %lu mV exceeds USB charging range - pin conflict detected", source_name, voltage_mv);
        return false;
    }
    if (voltage_mv < 2000) {   // Below realistic system operating voltage
        DLOG(TAG, "%s voltage %lu mV below realistic operating range", source_name, voltage_mv);
        return false;
    }
    
    // Log charging detection for T-Display S3
    if (voltage_mv > 4300) {
        DLOG(TAG, "%s charging detected: %lu mV (USB connected)", source_name, voltage_mv);
    }
#else
    // T5 and other boards: More conservative voltage range
    if (voltage_mv > 4500) {  // Above realistic max with battery connected
        WLOG(TAG, "%s voltage %lu mV exceeds expected range - pin conflict detected", source_name, voltage_mv);
        return false;
    }
    if (voltage_mv < 2000) {   // Below realistic operating voltage
        DLOG(TAG, "%s voltage %lu mV below realistic operating range", source_name, voltage_mv);
        return false;
    }
#endif
    return true;
}

/**
 * Common calibration function for both ULP and regular ADC readings
 * Applies hardware calibration if available, falls back to voltage conversion
 * @param raw_adc Raw ADC reading value
 * @return Calibrated voltage in millivolts, or 0 on error
 */
uint32_t calibrate_adc_raw(uint32_t raw_adc) {
    if (raw_adc == 0) {
        return 0; // Invalid reading
    }
    
    uint32_t calibrated_voltage = 0;
    
    // Apply hardware calibration if available
    if (adc_ctx.do_calibration && adc_ctx.adc1_cali_handle) {
        int temp_voltage; // ESP-IDF calibration API expects int*
        if (adc_cali_raw_to_voltage(adc_ctx.adc1_cali_handle, raw_adc, &temp_voltage) != ESP_OK) {
            // Fallback to voltage conversion without calibration
            calibrated_voltage = VOLTAGE_CONV((float)raw_adc);
        } else {
            calibrated_voltage = (uint32_t)temp_voltage;
        }
    } else {
        // No calibration handle available, use voltage conversion
        calibrated_voltage = VOLTAGE_CONV((float)raw_adc);
    }
    
    return calibrated_voltage;
}

/**
 * Helper function to get the most recent ADC reading as voltage
 * Consolidates duplicated voltage conversion logic
 */
static inline float get_recent_voltage_reading(void) {
    // printf("[%s] convert reading: %f V from %lu\n", __func__, 
    //     VOLTAGE_U32_TO_V((adc_ctx.result[get_adc_index(adc_ctx.result_index-1, 
    //     RESULT_SIZE)])), adc_ctx.result[get_adc_index(adc_ctx.result_index-1, RESULT_SIZE)]);
    return VOLTAGE_U32_TO_V(get_recent_reading(0));
}

/**
 * Helper function for voltage validation and clamping
 * Consolidates duplicated voltage validation logic for different board types
 */
float validate_and_clamp_voltage(float voltage, bool is_display_s3) {
    const float FALLBACK_VOLTAGE = FALLBACK_VOLTAGE_LILYGO;
    
    if (is_display_s3) {
        // T-Display S3 validation
        if (voltage > USB_VOLTAGE_MAX) {  // Above USB charging range suggests real pin conflict
            WLOG(TAG, "T-Display S3 continuous ADC voltage %f exceeds charging range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage < BATTERY_VOLTAGE_MIN) {  // Below realistic operating range
            WLOG(TAG, "T-Display S3 continuous ADC voltage %f below realistic range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage > CHARGING_VOLTAGE_THRESHOLD) {
            // Charging detected - this is normal behavior for T-Display S3
            DLOG(TAG, "T-Display S3 charging detected: %f V", voltage);
        }
    } else {
        // T5 and other boards: More conservative validation
        if (voltage > BATTERY_VOLTAGE_MAX_T5) {  // Above realistic range for T5 with battery
            WLOG(TAG, "Continuous ADC voltage %f exceeds T5 expected range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage < BATTERY_VOLTAGE_MIN) {  // Below realistic operating range
            WLOG(TAG, "Continuous ADC voltage %f below realistic range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        }
    }
    
    return voltage; // No clamping needed
}

/**
 * Enhanced battery state event posting with app mode context
 * Handles posting ESP events for battery state changes with intelligent filtering
 */
static void post_battery_state_event(adc_battery_state_t state, const char* source) {
    FUNC_ENTRY_ARGS(TAG, "source: %s, state: %d", source, state);
    uint32_t voltage_mv = get_recent_reading(0);
    
    // Check if we should filter events based on app mode
    bool should_filter = should_filter_charge_events();
    if (should_filter) {
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
    }
    
    // Post the event
    switch (state) {
        case ADC_BATTERY_LOW:
            WLOG(TAG, "%s detected low battery: %ld mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_LOW, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_HIGH:
            ILOG(TAG, "%s detected high battery: %ld mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_HIGH, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STARTED:
            ILOG(TAG, "%s detected charging started: %ld mV", source, voltage_mv);
            adc_charging_is_on = true;  // Update internal flag
            adc_lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STARTED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
            ILOG(TAG, "%s detected charging stopped: %ld mV", source, voltage_mv);
            adc_charging_is_on = false;  // Update internal flag
            adc_lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STOPPED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            ELOG(TAG, "%s detected critical low battery: %ld mV", source, voltage_mv);
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
    TREND_VOLATILE
} voltage_trend_t;

// Unified analysis result
typedef struct {
    uint32_t filtered_reading;    // Peak-filtered value
    voltage_trend_t trend;        // Current trend
    bool is_stable;              // Reading is stable
    int32_t rate_of_change;      // mV/reading trend
    bool is_charging_event;      // Potential charge state change
} adc_analysis_t;

/**
 * Elegant single-pass analysis
 * - Peak filtering
 * - Trend detection  
 * - Stability analysis
 * - Charge event detection
 */
static adc_analysis_t analyze_adc_readings(uint32_t current_reading, uint8_t available);

/**
 * Robust charge start detection with adaptive thresholds
 */
static bool detect_charge_start(uint32_t voltage_mv, adc_analysis_t analysis) {
        // CRITICAL FIX: Only detect charge start if voltage is actually RISING
    if (analysis.trend != TREND_RISING) {
        return false; // Cannot be charging if voltage is falling or stable!
    }
    
    // CRITICAL FIX: Require positive rate of change
    if (analysis.rate_of_change <= 0) {
        return false; // Cannot be charging if rate is negative or zero!
    }
    
    // Adaptive thresholds (only apply if above conditions are met)
    uint32_t rise_threshold;
    
    if (voltage_mv < 3600) rise_threshold = 100;    // Low battery: expect big jump
    else if (voltage_mv < 3800) rise_threshold = 70; // Medium battery  
    else if (voltage_mv < 4000) rise_threshold = 50; // High battery
    else rise_threshold = 40;                       // Very high battery
    
    // Only trigger if we have significant positive rise
    bool significant_rise = (analysis.rate_of_change >= rise_threshold);
    bool voltage_above_normal = (voltage_mv > 4050); // Additional confidence
    
    // For high batteries, require either significant rise OR moderate rise + above normal
    if (voltage_mv >= 3800) {
        return significant_rise || (analysis.rate_of_change >= 30 && voltage_above_normal);
    } else {
        // For low batteries, require significant rise only
        return significant_rise;
    }
}

/**
 * Robust charge stop detection with multiple safeguards
 */
static bool detect_charge_stop(uint32_t voltage_mv, adc_analysis_t analysis, uint32_t peak_voltage) {
    // Calculate drop from charging peak
    uint32_t drop_from_peak = peak_voltage - voltage_mv;
    
    // Charge stopped if: significant drop AND falling trend
    bool significant_drop = (drop_from_peak >= 80); // 80mV drop from peak
    bool falling_trend = (analysis.trend == TREND_FALLING);
    bool negative_roc = (analysis.rate_of_change < -20);
    
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
adc_battery_state_t get_battery_state(void) {
    static bool is_charging = false;
    static uint32_t last_charge_change_ms = 0;
    // static uint32_t charge_start_voltage = 0;
    static uint32_t charge_peak_voltage = 0;
    const uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    const uint32_t DEBOUNCE_MS = 3000;
    
    uint32_t voltage_mv = get_recent_reading(0);
    uint8_t available = adc_ctx.adc_buffer.count;
    adc_analysis_t analysis = analyze_adc_readings(voltage_mv, available);
    
    // ONE-TIME INIT: Set initial state from ULP detection
    if (!adc_initial_sync_done) {
        is_charging = adc_initial_charging_state;
        if (is_charging) {
            ILOG(TAG, "ADC: Using initial charging state from ULP");
        }
        adc_initial_sync_done = true;
    }

    printf("voltage:%lumV, charging:%d, trend:%d, roc:%ld\n",
           voltage_mv, is_charging, analysis.trend, analysis.rate_of_change);
    
    // 1. SAFETY FIRST: Critical low always triggers
    if (voltage_mv < BATTERY_CRITICAL_LOW_MV) {
        return ADC_BATTERY_CRITICAL_LOW;
    }
    
    // 2. CHARGE DETECTION (only when debounce period passed)
    if ((current_ms - last_charge_change_ms) >= DEBOUNCE_MS) {
        
        // --- CHARGING STARTED DETECTION ---
        if (!is_charging) {
            bool charge_start_detected = detect_charge_start(voltage_mv, analysis);
            if (charge_start_detected) {
                is_charging = true;
                last_charge_change_ms = current_ms;
                // charge_start_voltage = voltage_mv;
                charge_peak_voltage = voltage_mv;
                ILOG(TAG, "CHARGING STARTED: %lu mV (+%ld mV)", voltage_mv, analysis.rate_of_change);
                return ADC_BATTERY_CHARGING_STARTED;
            }
        }
        
        // --- CHARGING STOPPED DETECTION ---
        if (is_charging) {
            bool charge_stop_detected = detect_charge_stop(voltage_mv, analysis, charge_peak_voltage);
            if (charge_stop_detected) {
                is_charging = false;
                last_charge_change_ms = current_ms;
                // charge_start_voltage = 0;
                charge_peak_voltage = 0;
                ILOG(TAG, "CHARGING STOPPED: %lu mV (dropped from %lu mV)", voltage_mv, charge_peak_voltage);
                return ADC_BATTERY_CHARGING_STOPPED;
            }
            
            // Update peak voltage during charging
            if (voltage_mv > charge_peak_voltage) {
                charge_peak_voltage = voltage_mv;
            }
        }
    }
    
    // 3. STATE REPORTING
    if (is_charging) {
        return ADC_BATTERY_CHARGING_STARTED;
    } else {
        // Battery level reporting
        if (voltage_mv < BATTERY_LOW_MV) {
            return ADC_BATTERY_LOW;
        } else if (voltage_mv >= BATTERY_HIGH_MV) {
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
    
    if (available >= 3) {
        uint32_t current = get_recent_reading(0);
        uint32_t prev1 = get_recent_reading(1);
        uint32_t prev2 = get_recent_reading(2);
        
        // Use medium-term rate of change (more stable)
        result.rate_of_change = (int32_t)current - (int32_t)prev2;
        
        // FIXED: More conservative trend detection
        if (result.rate_of_change > 40) {
            result.trend = TREND_RISING;
        } else if (result.rate_of_change < -40) {
            result.trend = TREND_FALLING;
        } else {
            result.trend = TREND_STABLE;
        }
        
    } else if (available >= 2) {
        // Basic for small buffers
        result.rate_of_change = (int32_t)get_recent_reading(0) - (int32_t)get_recent_reading(1);
        result.trend = (result.rate_of_change > 30) ? TREND_RISING : 
                      (result.rate_of_change < -30) ? TREND_FALLING : TREND_STABLE;
    } else {
        result.trend = TREND_STABLE;
    }
    
    return result;
}

/**
 * Handle battery state machine for regular ADC mode (when ULP is disabled)
 * Called from adc_update() to process new voltage readings
 */
void handle_adc_battery_state(void) {
    adc_battery_state_t new_state = get_battery_state();
    uint32_t voltage_mv = get_recent_reading(0); // Get voltage from buffer
    FUNC_ENTRY_ARGS(TAG, " new_state: %s, last_state: %s", 
                    adc_battery_states_str[new_state], adc_battery_states_str[last_adc_battery_state]);

    // Only post events on state changes
    if (new_state != last_adc_battery_state) {
        // const char* state_names[] = {"NORMAL", "LOW", "HIGH", "CHARGING_STARTED", "CHARGING_STOPPED", "CRITICAL_LOW"};
        
        // Check if we should suppress state changes during transitions
        if (s_adc_events_suppressed) {
            // Always allow critical low battery state changes - safety first
            if (new_state != ADC_BATTERY_CRITICAL_LOW) {
                DLOG(TAG, "State change suppressed during transition: %s -> %s (%lu mV)", 
                     adc_battery_states_str[last_adc_battery_state], adc_battery_states_str[new_state], voltage_mv);
                return;  // Suppress the state change entirely
            }
        }
        
        DLOG(TAG, "State change: %s -> %s (%lu mV)", 
             adc_battery_states_str[last_adc_battery_state], adc_battery_states_str[new_state], voltage_mv);
        
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
        readings[i] = volt_read();
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

/**
 * Convert raw ADC reading to calibrated voltage (in volts)
 * This function can be used with any raw ADC value, including stored RTC values
 */
float adc_raw_to_voltage(uint32_t raw_adc_value) {
    // Use unified calibration logic
    uint32_t calibrated_voltage = calibrate_adc_raw(raw_adc_value);
    return (float)(calibrated_voltage / 1000UL); // Convert mV to V
}

/**
 * Get battery voltage optimized for display updates
 * Parameters: raw_adc_value (0 if not available), fallback_voltage, output pointer
 * Uses reference parameter to avoid return value overhead - more efficient for frequent calls
 */
void get_battery_voltage_for_display(uint32_t raw_adc_value, float fallback_voltage, float *voltage_out) {
#if defined(CONFIG_LOGGER_ADC_ENABLED)
    if (!voltage_out) return; // Safety check
    
    if (raw_adc_value > 0) {
        // Use provided raw value with efficient calibration
        *voltage_out = adc_raw_to_voltage(raw_adc_value);
    } else {
        // Fallback to provided voltage value
        *voltage_out = fallback_voltage;
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
static uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    FUNC_ENTRY(TAG);
    esp_err_t ret = ESP_FAIL;
    uint8_t calibrated = false;
    adc_cali_handle_t handle = NULL;
    if (!calibrated) {
        
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
        cali_mode = "Curve Fitting";
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
        cali_mode = "Line Fitting";
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
#endif
        DLOG(TAG,"[%s] calibration scheme version is %s", __func__, cali_mode);
        if (ret == ESP_OK) calibrated = true;
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
        DLOG(TAG,"[%s] Calibration Success", __func__);
    } else 
    if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        WLOG(TAG, "[%s] eFuse not burnt, skip software calibration", __func__);
    } else {
        ELOG(TAG, "[%s] Invalid arg or no memory", __func__);
    }
    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
    ILOG(TAG, "[%s]", __func__);
    DLOG(TAG, "[%s] deregister %s calibration scheme", __func__, cali_mode);
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_curve_fitting(handle)) {
        ELOG(TAG, "[%s] Failed to delete curve fitting scheme", __func__);
    }
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_line_fitting(handle)) {
        ELOG(TAG, "[%s] Failed to delete line fitting scheme", __func__);
    }
#endif
}

static uint32_t adc_read_raw() {
    // esp_err_t err = 0;
    int v = 0;
        // Regular mode: Take direct ADC readings
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
        // Regular mode: Use direct ADC readings
        if(!adc_ctx.adc1_handle || adc_oneshot_read(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &v)) {
            ELOG(TAG, "[%s] Failed to read ADC %d", __func__, _ADC_CHANNEL_0);
            return 0;
        }
        adc_ctx.adc_raw = v;
#endif
    if (adc_ctx.do_calibration) {
        if(!adc_ctx.adc1_cali_handle || adc_cali_raw_to_voltage(adc_ctx.adc1_cali_handle, adc_ctx.adc_raw, &v)) {
            ELOG(TAG, "[%s] Failed to convert", __func__);
            return 0;
        }
        adc_ctx.adc_voltage = v;
    }
    else adc_ctx.adc_voltage = adc_ctx.adc_raw;
    // TLOG(TAG, "[%s] ADC%d channel[%d]: raw: %lu, calibrated: %lu", __func__, _ADC_UNIT_0 + 1, _ADC_CHANNEL_0, adc_ctx.adc_raw, adc_ctx.adc_voltage);
    return adc_ctx.adc_voltage;
}

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

/* Low battery timer callback - handles final shutdown trigger */
static void adc_low_bat_timer_cb(void *arg) {
    if (battery_safety_mutex && xSemaphoreTake(battery_safety_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        WLOG(TAG, "Low battery timer expired - triggering shutdown callback");
        if (low_battery_callback) {
            low_battery_callback();
        }
        xSemaphoreGive(battery_safety_mutex);
    }
}





static void adc_update(void*arg) {
    FUNC_ENTRY(TAG);
    // Take 11 readings with 5ms delay between each for better stability on LilyGO T5 charging circuits
    // Increased sample count and delay to handle rapid voltage fluctuations during charging
    uint32_t reading = 0;
    if(adc_lock(100)) {
        reading = VOLTAGE_CONV(adc_read_count(11, 5));
        add_adc_reading(reading);
        adc_unlock();
    }
    
    handle_adc_battery_state();
    printf("Voltage: %lu mv\n", reading);
    // Integrated low battery monitoring and RTC voltage update - runs with every ADC update
    if (battery_safety_mutex && xSemaphoreTake(battery_safety_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float current_voltage = (float)reading / 1000.0f;  // Convert millivolts to volts
        printf ("Voltage: %.3f V\n", current_voltage);
        // Post voltage update event for main.c to handle RTC context updates  
        esp_event_post(ADC_EVENT, ADC_EVENT_UPDATE, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(50));
        
        // Low battery monitoring - trigger shutdown callback when battery is critically low
        static uint32_t low_bat_start_time = 0;
        
        if (current_voltage < minimum_battery_voltage) {
            uint32_t now = get_millis();
            
            if (low_bat_start_time == 0) {
                low_bat_start_time = now;
                WLOG(TAG, "Low battery detected: %.2fV < %.2fV - starting countdown", 
                     current_voltage, minimum_battery_voltage);
            } else if ((now - low_bat_start_time) > LOW_BAT_SEQUENCE_TIME_MS) {
                ELOG(TAG, "Battery critically low for %d seconds - triggering shutdown", 
                     (int)(LOW_BAT_SEQUENCE_TIME_MS / 1000));
                if (low_battery_callback) {
                    low_battery_callback();
                }
                low_bat_start_time = 0; // Reset to avoid repeated calls
            }
        } else {
            // Battery voltage is OK - reset countdown
            low_bat_start_time = 0;
        }
        
        xSemaphoreGive(battery_safety_mutex);
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

uint8_t adc_on_ac() { 
    return last_adc_battery_state == ADC_BATTERY_CHARGING_STARTED ? 1 : 0;
}

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

    // Setup calibration (works for both ULP and regular ADC)
    adc_ctx.do_calibration = adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN, &adc_ctx.adc1_cali_handle);
    if(adc_ctx.xMutex == NULL) adc_ctx.xMutex = xSemaphoreCreateMutex();
    if(adc_ctx.xMutex == NULL) {
        ELOG(TAG, "[%s] Failed to create mutex", __func__);
        return ESP_FAIL;
    }

    // Initialize regular ADC
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
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
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
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
    if(esp_timer_start_periodic(adc_ctx.adc_periodic_timer, MS_TO_US(500))) {
        ELOG(TAG, "[%s] Failed to start periodic timer", __func__);
        return ESP_FAIL;
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
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
    xTaskCreatePinnedToCore(adc_task, "ADC Task", (8*256), NULL, 0, &adc_ctx.adc_task_handle, 0);
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
    if (!battery_safety_mutex) {
        battery_safety_mutex = xSemaphoreCreateMutex();
        if (!battery_safety_mutex) {
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
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    if (adc_ctx.adc_periodic_timer) {
        esp_timer_stop(adc_ctx.adc_periodic_timer);
        esp_timer_delete(adc_ctx.adc_periodic_timer);
        adc_ctx.adc_periodic_timer = NULL;
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
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
    if (low_bat_timer) {
        esp_timer_stop(low_bat_timer);
        esp_timer_delete(low_bat_timer);
        low_bat_timer = NULL;
    }
    
    // Cleanup battery safety mutex
    if(battery_safety_mutex != NULL){
        vSemaphoreDelete(battery_safety_mutex);
        battery_safety_mutex = NULL;
    }
    low_battery_callback = NULL;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    if (adc_ctx.adc1_handle) {
        adc_oneshot_del_unit(adc_ctx.adc1_handle);
        adc_ctx.adc1_handle = NULL;
    }
#endif
    if (adc_ctx.do_calibration) {
        adc_ctx.do_calibration = 0;
        if (adc_ctx.adc1_cali_handle) {
            adc_calibration_deinit(adc_ctx.adc1_cali_handle);
            adc_ctx.adc1_cali_handle = NULL;
        }
    }
    return err;
}

float volt_read(void) {
    FUNC_ENTRY(TAG);
    float voltage = 0;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    // Always use the most recent reading, no smoothing
    if (adc_lock(100)) {
        voltage = get_recent_voltage_reading();
        adc_unlock();
    } else {
        voltage = get_recent_voltage_reading();
    }
    force_instant_voltage = false; // Clear flag after use (if set)
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    voltage = VOLTAGE_U32_TO_V((VOLTAGE_CONV((float)VOLTAGE_CONV_12(adc_ctx.adc_raw))));
#endif 
    
    // Additional validation for shared pin scenarios  
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    voltage = validate_and_clamp_voltage(voltage, true);
#else
    voltage = validate_and_clamp_voltage(voltage, false);
#endif

    DLOG(TAG, "[%s] adc_raw: %lu volt: %f", __func__, adc_ctx.adc_raw, voltage);
    return voltage;
}

/* Battery monitoring API - thread-safe access to battery data */
bool adc_check_battery_level(float minimum_voltage) {
    if (!adc_ctx.adc_initialized) {
        return true; // Default to safe if not initialized
    }
    
    float current_voltage = volt_read();
    return (current_voltage >= minimum_voltage);
}

void adc_set_low_battery_callback(void (*callback)(void)) {
    if (battery_safety_mutex && xSemaphoreTake(battery_safety_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        low_battery_callback = callback;
        xSemaphoreGive(battery_safety_mutex);
    } else {
        // Fallback assignment without mutex
        low_battery_callback = callback;
    }
}

void adc_set_minimum_battery_voltage(float voltage) {
    if (battery_safety_mutex && xSemaphoreTake(battery_safety_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        minimum_battery_voltage = voltage;
        ILOG(TAG, "Minimum battery voltage set to %.2fV", voltage);
        xSemaphoreGive(battery_safety_mutex);
    } else {
        minimum_battery_voltage = voltage;
    }
}

/* ADC event suppression functions - prevent false events during system transitions */
void adc_suppress_events(const char* reason) {
    s_adc_events_suppressed = true;
    s_adc_suppression_start_time = esp_timer_get_time() / 1000;  // Convert to ms
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
    int64_t current_time = esp_timer_get_time() / 1000;
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
        ESP_LOGD(TAG, "[%s] LCD charge notification flag cleared", __func__);
    }
    return was_set;
}

#endif // CONFIG_LOGGER_ADC_ENABLED
