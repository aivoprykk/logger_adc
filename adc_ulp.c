#include "adc_private.h"

static const char *TAG = "adc_ulp";

#if defined(CONFIG_ULP_COPROC_ENABLED)

#include "adc_ulp.h"

#include "ulp_program.h"
#include "ulp.h"

static bool ulp_live_snap_initialized = false;
static uint16_t initial_samples[ULP_ADC_HISTORY_SIZE] = {0};
static battery_snapshot_t ulp_live_snapshot = BATTERY_SNAPSHOT_DEFAULTS();

void ulp_live_snap_init(void) {
    if(ulp_live_snap_initialized) return;
    FUNC_ENTRYD(TAG);
    // battery_monitor_init(&ulp_live_snapshot.battery_monitor, 
    //     &battery_plateau, 
    //     &battery_slow_window, 
    //     &adc_current_battery_state,
    // #ifdef CONFIG_ULP_BUTTON_ENABLED
    //     &adc_current_button_state
    // #endif
    // );
    ulp_live_snap_initialized = true;
    ulp_live_snap_take();
}

battery_snapshot_t * ulp_live_snap_take_wait(uint32_t timeout_ms) {
    FUNC_ENTRYD(TAG);
    if(ulp_prog_is_initialized() && ulp_live_snap_initialized) {
        ulp_take_last_snapshot(&ulp_live_snapshot, timeout_ms);
    }
    return &ulp_live_snapshot;
}

battery_snapshot_t * ulp_live_snap_take(void) {
    return ulp_live_snap_take_wait(0);
}

battery_snapshot_t * ulp_live_snap_get(void) {
    FUNC_ENTRYD(TAG);
    return &ulp_live_snapshot;
}

const battery_monitor_t * ulp_live_snap_get_monitor(void) {
    FUNC_ENTRYD(TAG);
    return &ulp_live_snapshot.battery_monitor;
}

/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */

#if defined(CONFIG_ULP_BUTTON_ENABLED)
/* ULP st instruction always writes 32-bit, so .word packing doesn't work */
/* Must use .long and access as uint32_t */
#define ulp_button_press_counter_get() (ulp_button_press_counter & 0xFFF)
#define ulp_button_press_counter_set(val) (ulp_button_press_counter = (val) & 0xFFF)
#define ulp_button_last_result_get() (ulp_button_last_result & 0x1)
#define ulp_button_last_result_set(val) (ulp_button_last_result = (val) & 0x1)
#endif

void compute_and_store_ulp_thresholds(void) {
    FUNC_ENTRY(TAG);
    if(compute_and_store_thresholds() != ESP_OK) {
        ELOG(TAG, "ADC calibration handle not available, cannot compute ULP thresholds - using compile-time defaults");
        ULP_SET_U32(ulp_calibrated_voltage_3V2, ADC_LOW_THRESHOLD);
    }
}

// Check if same as last ADC wake reason (for suppression)
bool adc_ulp_same_adc_wake_reason(void) {
    adc_battery_state_t current_state = battery_get_current_battery_state();
    return (current_state != ADC_BATTERY_NORMAL) &&
           (current_state == battery_get_last_battery_state());
}

// Clear ULP wake sources - called on main CPU after wake
// preserve last status when immediate sleep is planned
void adc_ulp_clear_wake_sources(bool preserve_last_status) {
    FUNC_ENTRY(TAG);
    // ULP_SET_U32(ulp_battery_event_pending, 0);
    // Don't clear last_battery_status here - it's updated on wake based on source
    if(preserve_last_status) {
        ULP_SET_U32(ulp_last_battery_state, ulp_curr_battery_state);
    }
}

/**
 * Get ULP cycle count - safe accessor for main app
 * This properly reads ULP RTC memory using the ULP_GET_U32 macro
 */
uint32_t adc_ulp_get_cycle_count(void) {
    return ULP_GET_U32(ulp_cycle_count);
}

/**
 * Update last_wake_status based on current wake source
 * Call this on every wake to track previous ULP ADC wake for comparison
 * - If woken by ULP ADC: Save current status as last
 * - If woken by other source: Clear last status (no ADC wake to compare)
 */
int adc_ulp_after_wake(void) {
    FUNC_ENTRY(TAG);
    ULP_SET_U32(ulp_last_battery_state, 0);
    ulp_live_snap_init();
    return WAKE_SOURCE_BATTERY;
}

void ulp_take_last_snapshot(battery_snapshot_t *snap, uint32_t timeout_ms) {
    if (!ulp_prog_is_initialized() || !snap) return;
    uint32_t cycle_count = ULP_GET_U32(ulp_cycle_count);
    uint64_t snap_timestamp = esp_timer_get_time();
    while (timeout_ms && snap->snapshot_timestamp == cycle_count) {
        // Wait for new cycle count
        if (esp_timer_get_time() - snap_timestamp > TO_K_UL(CONFIG_ADC_CYCLE_TIME_MS)) {
            WLOG(TAG, "Timeout waiting for new ULP cycle count");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        cycle_count = ULP_GET_U32(ulp_cycle_count);
    }
    FUNC_ENTRY_ARGSD(TAG, "cycle_count=%lu, elapsed_time=%llu", ULP_GET_U32(ulp_cycle_count), (esp_timer_get_time()- snap_timestamp));
    snap->snapshot_timestamp = cycle_count;

    battery_monitor_t * bm = &snap->battery_monitor;
    bm->monitor_src = MON_SR_ULP; // ULP
    bm->voltage_raw = ULP_GET_U32(ulp_last_result);
    adc_plateau_t * plateau = bm->plateau;
    if (plateau) {
        plateau->last_sample = ULP_GET_U32(ulp_plateau_last_sample);
        plateau->count = ULP_GET_U32(ulp_plateau_count);
        plateau->direction = ULP_GET_U32(ulp_plateau_direction);
        plateau->adaptive_threshold = ULP_GET_U32(ulp_plateau_adaptive_threshold);
        plateau->processing = ULP_GET_U32(ulp_plateau_processing);
        plateau->delta_sum = ULP_GET_U32(ulp_plateau_delta_sum);
        plateau->delta_avg = ULP_GET_U32(ulp_plateau_delta_avg);
        plateau->delta = ULP_GET_U32(ulp_plateau_delta);
    }
    adc_running_avg_t * slow_window = bm->slow_window;
    if (slow_window) {
        slow_window->idx = ULP_GET_U32(ulp_slow_idx);
        slow_window->sum = ULP_GET_U32(ulp_slow_sum);
        slow_window->avg = ULP_GET_U32(ulp_slow_avg);
        slow_window->count = ULP_GET_U32(ulp_slow_count);
        if (slow_window->samples) { 
            uint16_t i = 0, j = slow_window->count < slow_window->size ? 
                                slow_window->count : slow_window->size;
            for (; i < j; i++) {
                slow_window->samples[i] = ULP_GET_ARR_U32(ulp_slow_samples, i);
            }
        }
    }
    adc_current_state_t * state = bm->battery_state;
    if (state) {
        state->curr = ULP_GET_U32(ulp_curr_battery_state);
        state->last = ULP_GET_U32(ulp_last_battery_state);
        // state->event_pending = ULP_GET_U32(ulp_battery_event_pending);
    }
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    state = bm->button_state;
    if (state) {
        state->curr = ULP_GET_U32(ulp_curr_button_state);
        state->last = ULP_GET_U32(ulp_last_button_state);
        // state->event_pending = ULP_GET_U32(ulp_button_event_pending);
    }
#endif
#if defined(DEBUG_ULP_VALUES)
    snapshot_dump(snap, " === ULP Live");
    ILOG(TAG, " === ULP Debug (delta_min=%hd < delta_abs=%hd) samples_needed=%lu adp_delta_path=%hd ===",
            (int16_t)ULP_GET_U32(ulp_debug_delta_min),
            (int16_t)ULP_GET_U32(ulp_delta_abs),
            ULP_GET_U32(ulp_plateau_samples_needed),
            (int16_t)ULP_GET_U32(ulp_debug_adp_delta_path)
         );
#if C_LOG_LEVEL <= LOG_DEBUG_NUM
    ulp_dump_calibration();
#endif
#endif
}

void ulp_dump_calibration(void) {
    if (!ulp_prog_is_initialized()) return;
    battery_calibration_t calib = {
        .voltage_3V2 = ULP_GET_U32(ulp_calibrated_voltage_3V2),
        .voltage_3V6 = ULP_GET_U32(ulp_calibrated_voltage_3V6),
        .voltage_3V8 = ULP_GET_U32(ulp_calibrated_voltage_3V8),
        .voltage_4V0 = ULP_GET_U32(ulp_calibrated_voltage_4V0),
        .voltage_4V1 = ULP_GET_U32(ulp_calibrated_voltage_4V1),
        .voltage_4V2 = ULP_GET_U32(ulp_calibrated_voltage_4V2)
    };
    battery_dump_calibration(&calib, " === ULP ");
}

/**
 * Get battery state using ULP variables when waking from ULP sleep
 * This function analyzes ULP ADC results to determine what triggered the wakeup
 * ULP ADC range: ~1800-2700 (vs main ADC: ~3300-4400), so we work with raw values
 */
uint8_t get_battery_state_from_ulp(void) {
    FUNC_ENTRY(TAG);
    return ULP_GET_U32(ulp_curr_battery_state);
}

#endif /* CONFIG_LOGGER_ADC_MODE_ULP */