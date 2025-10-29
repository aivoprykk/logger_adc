#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_MODE_ULP)

#include "ulp_program.h"
#include "ulp.h"
#include "common_log.h"

static const char *TAG = "adc_ulp";

RTC_DATA_ATTR static uint32_t rtc_stored_low_raw = 0;     // persists across deep-sleep (but NOT power-off)
RTC_DATA_ATTR static uint32_t rtc_stored_high_raw = 0;    // for hysteresis (clear threshold)

/* Helper: fetch three most-recent samples (current, prev1, prev2)
 * abstracts ULP vs non-ULP source so main analysis logic is unified.
 */
void ulp_get_three_samples(uint32_t *current, uint32_t *prev1, uint32_t *prev2)
{
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
    if (current) *current = ULP_GET_U32(ulp_last_result);
    uint32_t index = ULP_GET_U32(ulp_history_idx);
    if (prev1) *prev1 = ULP_GET_U32(ulp_history[((index - 2 + ULP_ADC_HISTORY_SIZE) % ULP_ADC_HISTORY_SIZE)]);
    if (prev2) *prev2 = ULP_GET_U32(ulp_history[(index - 3 + ULP_ADC_HISTORY_SIZE) % ULP_ADC_HISTORY_SIZE]);
    FUNC_ENTRY_ARGT(TAG, "ULP samples: current=%lu, prev1=%lu, prev2=%lu, index=%lu",
          (current ? *current : 0),
          (prev1 ? *prev1 : 0),
          (prev2 ? *prev2 : 0),
          index);
#else
    if (current) *current = get_recent_reading(0);
    if (prev1) *prev1 = get_recent_reading(1);
    if (prev2) *prev2 = get_recent_reading(2);
#endif
}

/**
 * Read ULP confirmation phase status if present.
 * Returns true if confirmation phase is active and fills out params.
 */
static bool ulp_confirmation_read_status(uint32_t *phase_out,
                                        uint32_t *sum_out,
                                        uint32_t *count_out,
                                        uint32_t *avg_out,
                                        uint32_t *direction_out)
{
    if (!phase_out || !sum_out || !count_out || !avg_out || !direction_out) return false;

    uint32_t phase = ULP_GET_U32(ulp_detection_phase);
    if (phase == 0) return false;  /* No active confirmation phase */

    /* Read confirmation phase fields */
    *phase_out = phase;
    *sum_out = ULP_GET_U32(ulp_confirmation_sum);
    *count_out = ULP_GET_U32(ulp_confirmation_count);
    *avg_out = ULP_GET_U32(ulp_confirmation_avg);
    *direction_out = ULP_GET_U32(ulp_detection_direction);

    FUNC_ENTRY_ARGSD(TAG, "Confirmation phase status read: phase=%lu, count=%lu", phase, *count_out);

    return true;
}

// New function to get detection phase info
uint8_t adc_ulp_get_detection_phase(void) {
    return (uint8_t)ULP_GET_U32(ulp_detection_phase);
}

/* Backward-compatible wrapper: returns false since snapshot logic removed */
static bool ulp_snapshot_read_and_consume(uint32_t *running_sum_out,
                                         uint32_t *history_idx_out,
                                         uint32_t *cycle_count_out,
                                         uint32_t *last_result_out,
                                         uint32_t *cum_change_out)
{
    /* Snapshot logic removed - always return false */
    return false;
}

/* Conversion wrapper: converts raw->mV using either adc_cali or legacy esp_adc_cal */
esp_err_t raw_to_mv_wrapper(int raw, uint32_t *voltage_mv)
{
    if (adc_ctx.do_calibration) {
        int tmp = 0;
        esp_err_t ret = adc_cali_raw_to_voltage(adc_ctx.cali_handle, raw, &tmp);
        if (ret == ESP_OK && voltage_mv) {
            *voltage_mv = (uint32_t)tmp;
        }
        return ret;
    }
    return ESP_ERR_INVALID_STATE;
}

static uint32_t find_raw_for_pin_mv(uint32_t pin_mv)
{
    FUNC_ENTRYD(TAG);
    uint32_t lo = 0;
    uint32_t hi = ADC_MAX_RAW;
    while (lo < hi) {
        uint32_t mid = (lo + hi) >> 1;
        uint32_t mv = 0;
        if (raw_to_mv_wrapper((int)mid, &mv) != ESP_OK) {
            // fallback: treat unknown as using linear scaling with DEFAULT_VREF to avoid infinite loop
            mv = (uint32_t)((uint64_t)mid * DEFAULT_VREF / ADC_MAX_RAW);
        }
        if (mv < pin_mv) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

void compute_and_store_ulp_thresholds(uint32_t desired_batt_mv)
{
    FUNC_ENTRY(TAG);
    if (adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN)) {
        // 2) Compute adc pin voltage (after divider)
        // In your repo: HIGH_RESISTOR, LOW_RESISTOR (both in ohms)
        uint32_t vpin_mv = VOLTAGE_CONV_MV_TO_ADC_ULL(desired_batt_mv);

        // 3) find the raw ADC value that maps to vpin_mv
        uint32_t raw_thresh = find_raw_for_pin_mv(vpin_mv);

        // 4) compute hysteresis/clear threshold (example: 5% above)
        uint32_t raw_clear = raw_thresh + (raw_thresh * HYSTERESIS_PERCENT) / 100;
        if (raw_clear > ADC_MAX_RAW) raw_clear = ADC_MAX_RAW;

        // 5) persist in RTC slow memory so it survives deep-sleep (use NVS if you need across power cycles)
        rtc_stored_low_raw = raw_thresh;
        rtc_stored_high_raw = raw_clear;

        if (raw_thresh == 0 || raw_thresh >= ADC_MAX_RAW) {
            ULP_SET_U32(ulp_low_threshold, ADC_LOW_THRESHOLD);
            goto err;
        } else {
            ULP_SET_U32(ulp_low_threshold, (raw_thresh));
        }
        // 6) ALSO write into ULP RAM symbol before starting ULP (see next snippet)
        ILOG(TAG, "Computed ULP low threshold: %lu (%lu) -> raw %lu (clear 5%% above: %lu)", vpin_mv, desired_batt_mv, raw_thresh, raw_clear);
        // adc_calibration_deinit();
    } else {
        err:
        ELOG(TAG, "ADC calibration handle not available, cannot compute ULP thresholds - using compile-time threshold");
        ULP_SET_U32(ulp_low_threshold, ADC_LOW_THRESHOLD);
    }
}

// Current state accessors - hybrid approach (2 packed variables)
uint8_t adc_get_ulp_wake_source(void) {
    return (ULP_GET_U32(ulp_curr_wake_status) & ULP_WAKE_CURRENT_SOURCE_MASK) >> ULP_WAKE_CURRENT_SOURCE_SHIFT;
}

// Last state accessors  
static inline uint8_t adc_get_ulp_last_wake_source(void) {
    return (ULP_GET_U32(ulp_last_wake_status) & ULP_WAKE_CURRENT_SOURCE_MASK) >> ULP_WAKE_CURRENT_SOURCE_SHIFT;
}

uint8_t adc_get_ulp_wake_reason(void) {
    return (ULP_GET_U32(ulp_curr_wake_status) & ULP_WAKE_CURRENT_ADC_MASK) >> ULP_WAKE_CURRENT_ADC_SHIFT;
}

uint8_t adc_get_ulp_last_wake_reason(void) {
    return (ULP_GET_U32(ulp_last_wake_status) & ULP_WAKE_CURRENT_ADC_MASK) >> ULP_WAKE_CURRENT_ADC_SHIFT;
}

static inline uint8_t adc_get_ulp_button_wake_reason(void) {
    return (ULP_GET_U32(ulp_curr_wake_status) & ULP_WAKE_CURRENT_BUTTON_MASK) >> ULP_WAKE_CURRENT_BUTTON_SHIFT;
}

static inline uint8_t adc_get_ulp_last_button_reason(void) {
    return (ULP_GET_U32(ulp_last_wake_status) & ULP_WAKE_CURRENT_BUTTON_MASK) >> ULP_WAKE_CURRENT_BUTTON_SHIFT;
}

// Your existing functions
bool adc_ulp_button_long_press_detected(void) {
    return (adc_get_ulp_wake_source() & ULP_WAKE_SOURCE_BUTTON) && 
           (adc_get_ulp_button_wake_reason() == ULP_BUTTON_WAKE_REASON_LONG_PRESS);
}

// Check if same as last ADC wake reason (for suppression)
bool adc_ulp_same_adc_wake_reason(void) {
    return (adc_get_ulp_wake_reason() != ULP_BAT_STATUS_NORMAL) &&
           (adc_get_ulp_wake_reason() == adc_get_ulp_last_wake_reason());
}

void adc_ulp_clear_wake_sources(void) {
    FUNC_ENTRY(TAG);
    ULP_SET_U32(ulp_curr_wake_status, 0);
    // Don't clear last_wake_status here - it's updated on wake based on source

    /* No snapshot logic to clear */
    DLOG(TAG, "Cleared curr_wake_status");
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
uint8_t adc_ulp_after_wake(void) {
    FUNC_ENTRY(TAG);
    debug_ulp_status();
    uint8_t wake_source = adc_get_ulp_wake_source();
    
    if (wake_source == ULP_WAKE_SOURCE_ADC) {
        /* Current wake was from ULP ADC - save for next wake comparison */
        uint32_t curr = ULP_GET_U32(ulp_curr_wake_status);
        ULP_SET_U32(ulp_last_wake_status, curr);
        DLOG(TAG, "Wake from ULP ADC: saved status (0x%02lX) as last for next comparison", curr & 0xFF);
    } else {
        /* Wake from other source (button/timer) - clear last status */
        ULP_SET_U32(ulp_last_wake_status, 0);
        DLOG(TAG, "Wake from %s: cleared last_wake_status (no ADC wake to compare)",
             adc_ulp_wake_sources_str(wake_source));
    }
    ulp_prog_set_main_cpu_running(true);
    return wake_source;
}

// Refactored to use ULP snapshots instead of live reads
bool ulp_history_snapshot_take(ulp_history_snapshot_t *out, bool compute_mad, int max_retries) {
    if (!out) return false;

    /* Check if snapshot is valid */
    uint32_t snapshot_valid = ULP_GET_U32(ulp_snapshot_valid);
    if (snapshot_valid == 0) {
        /* No valid snapshot available */
        out->valid_count = 0;
        SNAPSHOT_CLEAR_MAD_FLAG(out);
        SNAPSHOT_CLEAR_STATE(out);
        return false;
    }

    /* Read snapshot data */
    out->running_sum = ULP_GET_U32(ulp_snapshot_running_sum);
    out->history_idx = ULP_GET_U32(ulp_snapshot_history_idx);
    out->cycle_count = ULP_GET_U32(ulp_snapshot_cycle_count);
    out->valid_count = ULP_GET_U32(ulp_snapshot_valid_count);
    out->history_avg = ULP_GET_U32(ulp_snapshot_history_avg);
    out->last_sample = ULP_GET_U32(ulp_snapshot_last_sample);
    out->cum_change = ULP_GET_U32(ulp_snapshot_cum_change);
    // out->state = ULP_GET_U32(ulp_snapshot_state);

    /* Set flags */
    SNAPSHOT_SET_STATE(out);

    if (compute_mad) {
        out->mad = ULP_GET_U32(ulp_snapshot_mad);
        SNAPSHOT_SET_MAD_FLAG(out);
    } else {
        SNAPSHOT_CLEAR_MAD_FLAG(out);
    }

    DLOG(TAG, "ULP snapshot taken: valid=%lu, avg=%lu, last=%lu",
         out->valid_count, out->history_avg, out->last_sample);

    return true;
}

/**
 * Get snapshot confirmation_avg set during status change detection
 * Returns the confirmed average voltage when a status change was detected
 */
uint32_t adc_ulp_get_snapshot_confirmation_avg(void) {
    return ULP_GET_U32(ulp_snapshot_confirmation_avg);
}

/**
 * Get snapshot baseline_avg set during status change detection
 * Returns the baseline (running average) when a status change was detected
 */
uint32_t adc_ulp_get_snapshot_baseline_avg(void) {
    return ULP_GET_U32(ulp_snapshot_baseline_avg);
}

/**
 * Check if a history snapshot is valid
 */
bool adc_ulp_is_snapshot_valid(void) {
    return ULP_GET_U32(ulp_snapshot_valid) != 0;
}

/**
 * Get snapshot running sum
 */
uint32_t adc_ulp_get_snapshot_running_sum(void) {
    return ULP_GET_U32(ulp_snapshot_running_sum);
}

/**
 * Get snapshot history index
 */
uint32_t adc_ulp_get_snapshot_history_idx(void) {
    return ULP_GET_U32(ulp_snapshot_history_idx);
}

/**
 * Get snapshot cycle count
 */
uint32_t adc_ulp_get_snapshot_cycle_count(void) {
    return ULP_GET_U32(ulp_snapshot_cycle_count);
}

/**
 * Get snapshot valid count
 */
uint32_t adc_ulp_get_snapshot_valid_count(void) {
    return ULP_GET_U32(ulp_snapshot_valid_count);
}

/**
 * Get snapshot history average
 */
uint32_t adc_ulp_get_snapshot_history_avg(void) {
    return ULP_GET_U32(ulp_snapshot_history_avg);
}

/**
 * Get snapshot last sample
 */
uint32_t adc_ulp_get_snapshot_last_sample(void) {
    return ULP_GET_U32(ulp_snapshot_last_sample);
}

/**
 * Get snapshot cumulative change
 */
uint32_t adc_ulp_get_snapshot_cum_change(void) {
    return ULP_GET_U32(ulp_snapshot_cum_change);
}

/**
 * Get snapshot MAD (Mean Absolute Deviation)
 */
uint32_t adc_ulp_get_snapshot_mad(void) {
    return ULP_GET_U32(ulp_snapshot_mad);
}

/**
 * Get snapshot state
 */
uint32_t adc_ulp_get_snapshot_state(void) {
    return ULP_GET_U32(ulp_snapshot_state);
}

/**
 * Take a snapshot of current ULP history state for main CPU access
 * This captures the current running state into snapshot variables
 */
bool adc_ulp_take_history_snapshot(uint32_t snapshot_state) {
    FUNC_ENTRY(TAG);

    /* Capture current state with retry to avoid race conditions */
    int tries = 0;
    uint32_t before_cycle, after_cycle;
    uint32_t running_sum = 0;
    uint32_t history_idx = 0;
    uint32_t cycle_count = 0;
    uint32_t last_result = 0;
    uint32_t cum_change = 0;

    do {
        before_cycle = ULP_GET_U32(ulp_cycle_count);
        running_sum = ULP_GET_U32(ulp_running_sum);
        history_idx = ULP_GET_U32(ulp_history_idx);
        last_result = ULP_GET_U32(ulp_last_result);
        cum_change = ULP_GET_U32(ulp_cum_change);
        after_cycle = ULP_GET_U32(ulp_cycle_count);
        tries++;
    } while ((before_cycle != after_cycle) && (tries < 10));

    cycle_count = after_cycle;
    uint32_t valid_count = (cycle_count < ULP_ADC_HISTORY_SIZE) ? cycle_count : ULP_ADC_HISTORY_SIZE;
    uint32_t history_avg = 0;
    if (valid_count > 0) {
        history_avg = (valid_count == ULP_ADC_HISTORY_SIZE) ?
            (running_sum >> ULP_ADC_HISTORY_SHIFT) : (running_sum / valid_count);
    }

    /* Calculate MAD (Mean Absolute Deviation) */
    uint32_t mad = 0;
    if (valid_count > 0) {
        uint32_t oldest = (history_idx + ULP_ADC_HISTORY_SIZE - valid_count) % ULP_ADC_HISTORY_SIZE;
        for (uint32_t i = 0; i < valid_count; ++i) {
            uint32_t v = ULP_GET_ARR_U32(ulp_history, (oldest + i) % ULP_ADC_HISTORY_SIZE) & 0xFFF;
            mad += (v > history_avg) ? (v - history_avg) : (history_avg - v);
        }
        mad /= valid_count;
    }

    /* Store snapshot */
    ULP_SET_U32(ulp_snapshot_running_sum, running_sum);
    ULP_SET_U32(ulp_snapshot_history_idx, history_idx);
    ULP_SET_U32(ulp_snapshot_cycle_count, cycle_count);
    ULP_SET_U32(ulp_snapshot_valid_count, valid_count);
    ULP_SET_U32(ulp_snapshot_history_avg, history_avg);
    ULP_SET_U32(ulp_snapshot_last_sample, last_result);
    ULP_SET_U32(ulp_snapshot_cum_change, cum_change);
    ULP_SET_U32(ulp_snapshot_mad, mad);
    ULP_SET_U32(ulp_snapshot_state, snapshot_state);
    ULP_SET_U32(ulp_snapshot_valid, 1);  /* Mark snapshot as valid */

    DLOG(TAG, "ULP history snapshot taken: valid=%lu, avg=%lu, last=%lu, mad=%lu, state=%lu",
         valid_count, history_avg, last_result, mad, snapshot_state);

    return true;
}

void debug_snapshot(void) {
    TLOG(TAG, "ULP live - cycle_count: %lu, last_result: %lu, running_sum: %lu, history_idx: %lu, avg: %lu",
        ULP_GET_U32(ulp_cycle_count), ULP_GET_U32(ulp_last_result),
        ULP_GET_U32(ulp_running_sum), ULP_GET_U32(ulp_history_idx), ULP_GET_U32(ulp_running_sum) >> ULP_ADC_HISTORY_SHIFT);
    DLOG(TAG, "ULP confirmation - phase: %hhu, sum: %lu, count: %lu, avg: %lu, direction: %ld",
        adc_ulp_get_detection_phase(), ULP_GET_U32(ulp_confirmation_sum),
        ULP_GET_U32(ulp_confirmation_count), ULP_GET_U32(ulp_confirmation_avg),
        (int32_t)ULP_GET_U32(ulp_detection_direction));
    DLOG(TAG, "ULP adaptive - threshold: %lu, charging_active: %lu",
        ULP_GET_U32(ulp_adaptive_threshold), ULP_GET_U32(ulp_charging_active));
    TLOG(TAG, "ULP snapshots - confirmation_avg: %lu, baseline_avg: %lu",
        ULP_GET_U32(ulp_snapshot_confirmation_avg), ULP_GET_U32(ulp_snapshot_baseline_avg));
    TLOG(TAG, "ULP history snapshot - valid: %lu, running_sum: %lu, history_idx: %lu, cycle_count: %lu",
        ULP_GET_U32(ulp_snapshot_valid), ULP_GET_U32(ulp_snapshot_running_sum),
        ULP_GET_U32(ulp_snapshot_history_idx), ULP_GET_U32(ulp_snapshot_cycle_count));
    TLOG(TAG, "ULP history snapshot - valid_count: %lu, history_avg: %lu, last_sample: %lu, cum_change: %lu",
        ULP_GET_U32(ulp_snapshot_valid_count), ULP_GET_U32(ulp_snapshot_history_avg),
        ULP_GET_U32(ulp_snapshot_last_sample), ULP_GET_U32(ulp_snapshot_cum_change));
    TLOG(TAG, "ULP history snapshot - mad: %lu, state: %lu",
        ULP_GET_U32(ulp_snapshot_mad), ULP_GET_U32(ulp_snapshot_state));
}

static uint32_t time_prev = 0;
/**
 * Diagnostic function to debug ULP status
 */
void debug_ulp_status(void) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    if(!ulp_prog_is_initialized()) {
        return;
    }
    uint32_t time_now = (esp_timer_get_time() / 1000UL);
    printf("---- ULP Status Dump ---- %lu cycle: %lu\n", time_prev ? time_now-time_prev : 0, ULP_GET_U32(ulp_cycle_count));
    time_prev = time_now;
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    DLOG(TAG, "ULP Bat - charging_active: %lu", ULP_GET_U32(ulp_charging_active));
    debug_snapshot();
    uint32_t manual_sum = 0;
    for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        manual_sum += ULP_GET_ARR_U32(ulp_history, i);
    }
    uint32_t manual_avg = manual_sum / ULP_ADC_HISTORY_SIZE;
    uint32_t ulp_avg = ULP_GET_U32(ulp_running_sum) >> ULP_ADC_HISTORY_SHIFT;
    TLOG(TAG, "ULP Bat - manual_avg: %lu, ulp_running_sum avg: %lu", manual_avg, ulp_avg);
    char a[128] = {0};
    char *p = a;
    for (int i = 0, j = ULP_GET_U32(ulp_history_idx); i < ULP_ADC_HISTORY_SIZE; i++) {
        p += sprintf(p, "%lu", ULP_GET_ARR_U32(ulp_history, i));
        if(i == (j == 0 ? ULP_ADC_HISTORY_SIZE - 1 : j - 1)) {
            p += sprintf(p, "<-idx");
        }
        if (i < (ULP_ADC_HISTORY_SIZE - 1)) p += sprintf(p, ", ");
    }
    DLOG(TAG, "ULP Bat - ulp_history: [%s]", &a[0]);
    TLOG(TAG, "ULP Bat - low_thresh=%lu, rapid_change_thresh=%d ", ULP_GET_U32(ulp_low_threshold), ADC_RAPID_CHANGE_THRESHOLD);
#endif
    
    // Display wake status - hybrid approach (2 packed variables)
    uint32_t curr_status = ULP_GET_U32(ulp_curr_wake_status);
    uint32_t last_status = ULP_GET_U32(ulp_last_wake_status);
    
    uint8_t curr_source = (curr_status >> ULP_WAKE_CURRENT_SOURCE_SHIFT) & 0x3;
    uint8_t curr_bat = (curr_status >> ULP_WAKE_CURRENT_ADC_SHIFT) & 0x7;
    uint8_t curr_button = (curr_status >> ULP_WAKE_CURRENT_BUTTON_SHIFT) & 0x7;
    uint8_t last_source = (last_status >> ULP_WAKE_CURRENT_SOURCE_SHIFT) & 0x3;
    uint8_t last_bat = (last_status >> ULP_WAKE_CURRENT_ADC_SHIFT) & 0x7;
    uint8_t last_button = (last_status >> ULP_WAKE_CURRENT_BUTTON_SHIFT) & 0x7;
    
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    TLOG(TAG, "ULP Btn - enabled: gpio: %d, press_counter: addr=%p, value=%lu, last_result: addr=%p, value=%lu",
        CONFIG_ULP_BUTTON_GPIO, &ulp_button_press_counter, ulp_button_press_counter_get(),
        &ulp_button_last_result, ulp_button_last_result_get());
    TLOG(TAG, "ULP Btn - press in progress: counter=%lu (0=no press, >0=pressing, threshold=%d)",
           ulp_button_press_counter_get(), ULP_LONG_PRESS_CYCLES);
#endif
    // DLOG(TAG, " - ULP Wake Status (hybrid: 2 packed vars, curr=0x%02lX, last=0x%02lX):", curr_status & 0xFF, last_status & 0xFF);
    DLOG(TAG, "ULP - Current: Source=%s (%hhu), Bat_reason=%s (%hhu), Button_reason=%s (%hhu)",
           adc_ulp_wake_sources_str(curr_source), curr_source,
           adc_ulp_adc_wake_reasons_str(curr_bat), curr_bat,
           adc_ulp_button_wake_reasons_str(curr_button), curr_button);
    DLOG(TAG, "ULP - Last:    Source=%s (%hhu), Bat_reason=%s (%hhu), Button_reason=%s (%hhu)",
           adc_ulp_wake_sources_str(last_source), last_source,
           adc_ulp_adc_wake_reasons_str(last_bat), last_bat,
           adc_ulp_button_wake_reasons_str(last_button), last_button);
#endif
    printf("-------------------------\n");
}

/* Helper: compute Mean Absolute Deviation (MAD) of ULP history (raw units) */
uint32_t compute_ulp_history_mad(void) {
    uint32_t history_avg = 0;
    for (uint8_t i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        history_avg += (ULP_GET_ARR_U32(ulp_history, i) & 0xFFF);
    }
    history_avg /= ULP_ADC_HISTORY_SIZE;

    uint32_t mad = 0;
    for (uint8_t i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        uint32_t v = (ULP_GET_ARR_U32(ulp_history, i) & 0xFFF);
        mad += (v > history_avg) ? (v - history_avg) : (history_avg - v);
    }
    mad /= ULP_ADC_HISTORY_SIZE;
    return mad;
}

/**
 * Get battery state using ULP variables when waking from ULP sleep
 * This function analyzes ULP ADC results to determine what triggered the wakeup
 * ULP ADC range: ~1800-2700 (vs main ADC: ~3300-4400), so we work with raw values
 */
adc_battery_state_t get_battery_state_from_ulp(void) {
    FUNC_ENTRY(TAG);
    
    uint8_t wake_source = adc_get_ulp_wake_source();
    uint8_t wake_reason = adc_get_ulp_wake_reason();
    
    ILOG(TAG, "ULP wake: source=%d, reason=%d, detection_phase=%d", 
         wake_source, wake_reason, adc_ulp_get_detection_phase());
    
    if (wake_source == ULP_WAKE_SOURCE_ADC) {
        // Detection analysis no longer available (snapshot variables removed for size optimization)
        
        switch (wake_reason) {
            case ADC_ULP_BATTERY_CHARGING_STARTED:
                ILOG(TAG, "ULP: Charge start detected (two-phase confirmed)");
                return ADC_BATTERY_CHARGING_STARTED;
            case ADC_ULP_BATTERY_CHARGING_STOPPED:
                ILOG(TAG, "ULP: Charge stop detected (two-phase confirmed)");
                return ADC_BATTERY_CHARGING_STOPPED;
 #if 1 == 0              
            case ULP_ADC_WAKE_REASON_RAPID_CHG:
                // Legacy rapid change - use direction from snapshot if available
                if (has_detection_data) {
                    if (direction) {
                        ILOG(TAG, "ULP: Legacy rapid change -> Charge start (from direction)");
                        return ADC_BATTERY_CHARGING_STARTED;
                    } else {
                        ILOG(TAG, "ULP: Legacy rapid change -> Charge stop (from direction)");
                        return ADC_BATTERY_CHARGING_STOPPED;
                    }
                }
                break;
#endif
            case ADC_ULP_BATTERY_CRITICAL_LOW:
                ILOG(TAG, "ULP: Low battery detected");
                return ADC_BATTERY_LOW;
            default:
                ILOG(TAG, "ULP: Unknown ADC reason %d", wake_reason);
                break;
        }
    }
    // Check if we're in detection phase (waiting confirmation)
    uint8_t detection_phase = adc_ulp_get_detection_phase();
    if (detection_phase == 1) {
        DLOG(TAG, "ULP in detection phase (waiting confirmation)");
    }

    return ADC_BATTERY_NORMAL;
}

// Snapshot accessor functions for confirmed values
uint32_t ulp_get_snapshot_confirmation_avg(void)
{
    return ULP_GET_U32(ulp_snapshot_confirmation_avg);
}

uint32_t ulp_get_snapshot_baseline_avg(void)
{
    return ULP_GET_U32(ulp_snapshot_baseline_avg);
}

#endif /* CONFIG_ULP_COPROC_ENABLED */