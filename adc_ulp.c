#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_MODE_ULP)

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "hal/rtc_hal.h"
#include "esp_sleep.h"
#include "esp_system.h"  /* For esp_reset_reason() */
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_adc.h"

static const char *TAG = "adc_ulp";
RTC_DATA_ATTR bool ulp_initialized = false;
RTC_DATA_ATTR bool ulp_adc_initialized = false;

/* ULP snapshot symbols (defined in ULP RTC fast memory by adc.S)
 * The ULP build embeds these labels and the component tooling exposes them
 * to the main firmware with an "ulp_" prefix (e.g. running_sum -> ulp_running_sum).
 * Use the ulp_snapshot_* names here so the C linker sees the symbols. */
extern uint32_t ulp_snapshot_valid;
extern uint32_t ulp_snapshot_running_sum;
extern uint32_t ulp_snapshot_history_idx;
extern uint32_t ulp_snapshot_cycle_count;
extern uint32_t ulp_snapshot_last_result;
extern uint32_t ulp_snapshot_mad;
extern uint32_t ulp_snapshot_state;

/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */
extern uint32_t ulp_curr_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_last_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_entry;

extern uint32_t ulp_low_threshold;

/* ULP binary references */
extern const uint8_t ulp_battery_bin_start[] asm("_binary_ulp_battery_bin_start");
extern const uint8_t ulp_battery_bin_end[]   asm("_binary_ulp_battery_bin_end");

// Access history array from ULP
#define ADC_THRESHOLD_TRIGGER 1

/* Map ADC channel to GPIO pin number */
static int adc_channel_to_gpio(adc_channel_t channel) {
    switch (channel) {
        case ADC_CHANNEL_0: return GPIO_NUM_36;
        case ADC_CHANNEL_1: return GPIO_NUM_37;
        case ADC_CHANNEL_2: return GPIO_NUM_38;
        case ADC_CHANNEL_3: return GPIO_NUM_39;
        case ADC_CHANNEL_4: return GPIO_NUM_32;
        case ADC_CHANNEL_5: return GPIO_NUM_33;
        case ADC_CHANNEL_6: return GPIO_NUM_34;
        case ADC_CHANNEL_7: return GPIO_NUM_35;
        default:
            ELOG(TAG, "Unsupported ADC channel %d", channel);
            return -1;
    }
}

static void adc_ulp_init_rtc_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }
    // Configure button GPIO for ULP use
    rtc_gpio_init(rtc_gpio);
    rtc_gpio_set_direction(rtc_gpio, RTC_GPIO_MODE_INPUT_ONLY);
    switch(rtc_gpio) {
#ifdef CONFIG_ULP_BUTTON_ENABLED
        case CONFIG_ULP_BUTTON_GPIO:
            rtc_gpio_pullup_en(rtc_gpio);
            rtc_gpio_pulldown_dis(rtc_gpio);
            break;
#endif
        default:
            rtc_gpio_pulldown_dis(rtc_gpio);
            rtc_gpio_pullup_dis(rtc_gpio);
            break;
    }
    //
    ILOG(TAG, "GPIO pin %d configured for RTC.", rtc_gpio);
}


static void adc_ulp_uninit_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }
    
    /* Deinit RTC mode first */
    rtc_gpio_deinit(rtc_gpio);
    
    /* GPIO 34-39 on ESP32 are input-only - skip reset and hold operations */
    if (rtc_gpio >= 34 && rtc_gpio <= 39) {
        ILOG(TAG, "GPIO pin %d (input-only) cleared RTC.", rtc_gpio);
        return;
    }
    
    /* Reset pin to default state (safe for output-capable GPIOs) */
    gpio_reset_pin(rtc_gpio);
    gpio_hold_dis(rtc_gpio);
    rtc_gpio_hold_dis(rtc_gpio);
    
    ILOG(TAG, "GPIO pin %d cleared RTC.", rtc_gpio);
}

static void configure_adc_pad(void)
{
    FUNC_ENTRY(TAG);
    
    int adc_gpio = adc_channel_to_gpio(_ADC_CHANNEL_0);
    if (adc_gpio == -1) {
        return;
    }
    
    // Configure ADC pad for ULP use
    adc_ulp_init_rtc_pin(adc_gpio);

    ILOG(TAG, "ADC GPIO%d (channel %d) configured for ULP", adc_gpio, _ADC_CHANNEL_0);
}

static void adc_ulp_init_pins(void)
{
    FUNC_ENTRY(TAG);
    configure_adc_pad();
#ifdef CONFIG_ULP_BUTTON_ENABLED
    adc_ulp_init_rtc_pin(CONFIG_ULP_BUTTON_GPIO);
#endif
}

void adc_ulp_uninit_pins(void)
{
    FUNC_ENTRY(TAG);
#ifdef CONFIG_ULP_BUTTON_ENABLED
    adc_ulp_uninit_pin(CONFIG_ULP_BUTTON_GPIO);
#endif
}

/**
 * Read ULP snapshot if present and consume it (clear valid flag).
 * Returns true if snapshot was present and filled into out params.
 */
static bool ulp_snapshot_read_and_consume_full(uint32_t *running_sum_out,
                                               uint32_t *history_idx_out,
                                               uint32_t *cycle_count_out,
                                               uint32_t *last_result_out,
                                               uint32_t *mad_out,
                                               uint32_t *state_out)
{
    // FUNC_ENTRY(TAG);
    if (!running_sum_out || !history_idx_out || !cycle_count_out || !last_result_out) return false;
    /* mad_out and state_out are optional (may be NULL) */
    uint32_t valid = ULP_GET_U32(ulp_snapshot_valid);
    if (valid == 0) return false;
    /* Read snapshot fields (ULP wrote these before halting) */
        *running_sum_out = ULP_GET_U32(ulp_snapshot_running_sum);
        *history_idx_out = ULP_GET_U32(ulp_snapshot_history_idx);
        *cycle_count_out = ULP_GET_U32(ulp_snapshot_cycle_count);
        *last_result_out = ULP_GET_U32(ulp_snapshot_last_result) & 0xFFF;
        if (mad_out) {
            *mad_out = ULP_GET_U32(ulp_snapshot_mad) & 0xFFFF; /* ULP writes MAD in lower 16 bits */
        }
        if (state_out) {
            *state_out = ULP_GET_U32(ulp_snapshot_state) & 0xFF; /* small enum in lower 8 bits */
    }
    /* Consume snapshot so next wake won't reuse stale data */
    ULP_SET_U32(ulp_snapshot_valid, 0);
    FUNC_ENTRY_ARGSD(TAG, "ULP snapshot taken, cycle_count: %lu, last_result: %lu ", *cycle_count_out, *last_result_out);
    return true;
}

/* Backward-compatible wrapper: original callers expect 4 args. */
static bool ulp_snapshot_read_and_consume(uint32_t *running_sum_out,
                                         uint32_t *history_idx_out,
                                         uint32_t *cycle_count_out,
                                         uint32_t *last_result_out)
{
    return ulp_snapshot_read_and_consume_full(running_sum_out, history_idx_out, cycle_count_out, last_result_out, NULL, NULL);
}

RTC_DATA_ATTR static uint32_t rtc_stored_low_raw = 0;     // persists across deep-sleep (but NOT power-off)
RTC_DATA_ATTR static uint32_t rtc_stored_high_raw = 0;    // for hysteresis (clear threshold)

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
    DLOG(TAG, "Mapped pin voltage %lu mV to raw %lu", pin_mv, lo);
    return lo;
}

static void compute_and_store_ulp_thresholds(uint32_t desired_batt_mv)
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
        ILOG(TAG, "Computed ULP low threshold: %lu (%lu) -> raw %lu (clear at %lu)", vpin_mv, desired_batt_mv, raw_thresh, raw_clear);
        // adc_calibration_deinit();
    } else {
        err:
        ELOG(TAG, "ADC calibration handle not available, cannot compute ULP thresholds - using compile-time threshold");
        ULP_SET_U32(ulp_low_threshold, ADC_LOW_THRESHOLD);
    }
}

/**
 * Initialize ULP ADC hardware for battery monitoring
 * Can be called multiple times - exits early if already initialized
 * Only uses locking if main ADC is initialized (lock exists)
 */
esp_err_t init_ulp_adc(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;
    
    /* Exit early if already initialized */
    if (ulp_adc_initialized) {
        DLOG(TAG, "ULP ADC already initialized, skipping");
        return ESP_OK;
    }
    
    /* Try to acquire lock if ADC is initialized, otherwise just proceed */
    bool locked = adc_lock(1000);
    if (!locked) {
        DLOG(TAG, "ADC lock not available (main ADC not initialized), proceeding without lock");
    }
    
    /* Initialize ULP ADC hardware */
    ulp_adc_cfg_t adc_cfg = {
        .adc_n = _ADC_UNIT_0,     // Use same unit as regular ADC
        .channel = _ADC_CHANNEL_0, // Use same channel as regular ADC  
        .atten = _ADC_ATTEN,      // Use same attenuation as regular ADC
        .width = _ADC_BITWIDTH,   // Use same bitwidth as regular ADC (only for ADC1)
        .ulp_mode = ADC_ULP_MODE_FSM, // Explicitly specify FSM mode for ESP32 (not RISC-V)
    };
    
    err = ulp_adc_init(&adc_cfg);
    if (err != ESP_OK) {
        ELOG(TAG, "ULP ADC init failed: %s", esp_err_to_name(err));
        if (locked) adc_unlock();
        return err;
    }

    /* Configure GPIO pins for ULP use */
    adc_ulp_init_pins();
    
    ulp_adc_initialized = true;
    ILOG(TAG, "ULP ADC hardware initialized");
    
    if (locked) adc_unlock();
    return ESP_OK;
}

/**
 * Deinitialize ULP ADC hardware
 * Can be called multiple times - exits early if not initialized
 * Only uses locking if main ADC is initialized (lock exists)
 */
void deinit_ulp_adc(void) {
    FUNC_ENTRY(TAG);
    
    /* Exit early if not initialized */
    if (!ulp_adc_initialized) {
        DLOG(TAG, "ULP ADC not initialized, nothing to deinit");
        return;
    }
    
    /* Try to acquire lock if ADC is initialized, otherwise just proceed */
    bool locked = adc_lock(1000);
    if (!locked) {
        DLOG(TAG, "ADC lock not available (main ADC not initialized), proceeding without lock");
    }
    
    /* Clear flag first to prevent re-entry if deinit fails */
    ulp_adc_initialized = false;
    
    esp_err_t err = ulp_adc_deinit();
    if (err != ESP_OK) {
        /* Log error but continue cleanup - don't restore flag */
        ELOG(TAG, "ULP ADC deinit failed: %s", esp_err_to_name(err));
    } else {
        ILOG(TAG, "ULP ADC hardware deinitialized");
    }
    
    /* Uninitialize the ADC GPIO pin from RTC mode (safe even if deinit failed) */
    int adc_gpio = adc_channel_to_gpio(_ADC_CHANNEL_0);
    if (adc_gpio != -1) {
        adc_ulp_uninit_pin(adc_gpio);
    }
    
    if (locked) adc_unlock();
}

/**
 * Initialize ULP program - load binary once at power-up
 * This runs only once when powered up (ulp_initialized flag prevents reload)
 * IMPORTANT: On power-on reset, we must reload the ULP binary because RTC fast
 * memory is cleared. The ulp_initialized flag persists in RTC slow memory, so we
 * check reset reason to detect true power-on and force reload.
 */
esp_err_t init_ulp_program(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;

    /* Check reset reason - on power-on reset, RTC fast memory (ULP code) is cleared
     * but RTC slow memory (ulp_initialized flag) persists. We must reload binary. */
    esp_reset_reason_t reset_reason = esp_reset_reason();
    bool force_reload = (reset_reason == ESP_RST_POWERON);
    
    if (force_reload) {
        ILOG(TAG, "Power-on reset detected - forcing ULP binary reload (ulp_initialized=%d)", ulp_initialized);
        ulp_initialized = false;  // Clear flag to force reload
    }

    /* Check if ULP binary already loaded - run only once at power-up */
    if (ulp_initialized) {
        DLOG(TAG, "ULP program already loaded, skipping");
        return ESP_OK;
    }

    ILOG(TAG, "Loading ULP program binary (reset_reason=%d)...", reset_reason);
    
    /* Load ULP binary - only once at power-up */
    const size_t ulp_prog_size_bytes = ulp_battery_bin_end - ulp_battery_bin_start;
    const size_t ulp_prog_size_words = ulp_prog_size_bytes / sizeof(uint32_t);

    err = ulp_load_binary(0, ulp_battery_bin_start, ulp_prog_size_words);
    if (err != ESP_OK) {
        ELOG(TAG, "Failed to load ULP program: %s", esp_err_to_name(err));
        return err;
    }

    compute_and_store_ulp_thresholds(BATTERY_CRITICAL_LOW_MV);
    
    /* First boot: initialize last_wake_status to 0 (no previous wake) */
    ULP_SET_U32(ulp_last_wake_status, 0);
    
    ulp_initialized = true;
    ILOG(TAG, "ULP program loaded (%u bytes, %u words), last_wake_status initialized to 0", 
         ulp_prog_size_bytes, ulp_prog_size_words);
    DLOG(TAG, "Raw ULP variable check (first boot - binary loaded):");
    DLOG(TAG, "  Thresholds: low=%lu, rapid_change=%d", 
           ULP_GET_U32(ulp_low_threshold), ADC_RAPID_CHANGE_THRESHOLD);
    DLOG(TAG, "  last_result addr=%p, value=0x%08lX (%lu)", 
            &ulp_last_result, ULP_GET_U32(ulp_last_result), ULP_GET_U32(ulp_last_result));
    
    return ESP_OK;
}

void start_ulp_program(void)
{
    FUNC_ENTRY(TAG);
    /* Clear any stale ULP snapshot at start to avoid misinterpreting old data */
    ULP_SET_U32(ulp_snapshot_valid, 0);
    ULP_SET_U32(ulp_snapshot_running_sum, 0);
    ULP_SET_U32(ulp_snapshot_history_idx, 0);
    ULP_SET_U32(ulp_snapshot_cycle_count, 0);
    ULP_SET_U32(ulp_snapshot_last_result, 0);
    ULP_SET_U32(ulp_snapshot_mad, 0);
    ULP_SET_U32(ulp_snapshot_state, 0);

    ILOG(TAG, "Starting ULP program (fresh sleep - clearing ADC history)...");
    
    /* Note: init_ulp_program() is now called in wakeup_init() at boot, not here.
     * This ensures ULP binary is loaded before any sleep operations. */
    // adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN);
    init_ulp_adc();      // Initialize ULP ADC hardware (with locking)
    // rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Clear current wake status and ADC history for fresh sleep cycle.
     * ADC history must be cleared because battery voltage changed during wake time
     * (charging/discharging while CPU was active). The ULP needs fresh measurements
     * to accurately detect charging events in the new sleep cycle. */
    ULP_SET_U32(ulp_curr_wake_status, 0);
    // ULP_SET_U32(ulp_last_result, 0);
    // ULP_SET_U32(ulp_cum_change, 0);
    // ULP_SET_U32(ulp_history_idx, 0);
    // ULP_SET_U32(ulp_running_sum, 0);
    // for (int i = 0; i < 4; i++) {
    //     ULP_SET_ARR_U32(ulp_history, i, 0);
    // }
    
    DLOG(TAG, "Cleared: curr_wake_status, ADC history (preserved cycle_count=%lu)", 
         ULP_GET_U32(ulp_cycle_count));
    
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    /* Reset button press counter - button events are handled immediately, not accumulated */
    ulp_button_press_counter = 0;
    DLOG(TAG, "Cleared: button_press_counter");
#endif

    /* Preserve last_wake_status - it's updated on wake, not here */
    uint32_t last_wake = ULP_GET_U32(ulp_last_wake_status);
    ILOG(TAG, "Preserved last_wake_status=0x%02lX for next wake comparison", last_wake & 0xFF);

    /* Configure ULP timer for periodic wakeup during deep sleep
     * ULP cycle time is 125ms (configured in adc.S), so set wakeup period to match.
     * Note: The ULP assembly has internal wait loops, but this timer is what triggers
     * the ULP to run during deep sleep. Without this, ULP only runs while CPU is awake! */
    ulp_set_wakeup_period(0, TO_K_UL(ULP_CYCLE_TIME_MS));  /* 125ms = 125000 microseconds */
    
    /* Start the program */
    uint32_t cycle_before = ULP_GET_U32(ulp_cycle_count);
    DLOG(TAG, "Calling ulp_run() with 125ms timer - cycle_count before=%lu", cycle_before);
    
    esp_err_t err = ulp_run((uint32_t*)&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        ELOG(TAG, "Failed to start ULP program: %s", esp_err_to_name(err));
        return;
    }
    
    /* Give ULP time to start and increment cycle_count (one cycle = ~125ms, but may start immediately) */
    vTaskDelay(pdMS_TO_TICKS(ULP_CYCLE_TIME_MS));
    uint32_t cycle_after = ULP_GET_U32(ulp_cycle_count);
    ILOG(TAG, "ULP program started: cycle_count before=%lu, after_200ms=%lu (delta=%ld)", 
         cycle_before, cycle_after, (int32_t)(cycle_after - cycle_before));
    if (cycle_after == cycle_before) {
        WLOG(TAG, "WARNING: ULP cycle_count did NOT increment after ulp_run() - ULP may not be running!");
    }
    
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
    debug_ulp_status();
#endif
}

/**
 * @brief Resume ULP program after ULP wake without clearing ADC history
 * 
 * This function is called when going back to sleep after a ULP wake event.
 * It restarts ULP execution using ulp_run() but preserves all ADC monitoring
 * state (history[], running_sum, etc.) so rapid change detection continues
 * to work immediately. The ULP was halted by the 'halt' instruction and just
 * needs to resume execution from the entry point.
 * 
 * Critical difference from start_ulp_program():
 * - Does NOT clear ADC history (battery voltage hasn't changed - we just woke
 *   to process the ULP event and are immediately going back to sleep)
 * - Does NOT call init_ulp_adc() (hardware already initialized)
 * - Just calls ulp_run() to resume monitoring from where it left off
 */
void resume_ulp_program(void)
{
    FUNC_ENTRY(TAG);
    
    ILOG(TAG, "Resuming ULP program (post-ULP-wake - preserving ADC history)...");
    
    /* Only clear current wake status - the main CPU has processed it.
     * Preserve ALL monitoring state (cycle_count, history[], running_sum, etc.)
     * because we're just resuming the ULP after a brief wake to handle an event.
     * The battery voltage hasn't changed significantly during the short wake period. */
    ULP_SET_U32(ulp_curr_wake_status, 0);
    
    /* CRITICAL: Add delay to ensure RTC memory write is visible to ULP before restart.
     * Without this, ULP may start before curr_wake_status clear propagates, see non-zero
     * wake status, jump to adc_frozen, and halt forever. */
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Ensure ULP timer is configured for periodic wakeup during deep sleep.
     * This is critical - without it, ULP only runs while main CPU is awake! */
    ulp_set_wakeup_period(0, 125000);  /* 125ms = 125000 microseconds */
    
    DLOG(TAG, "Resuming with: cycle_count=%lu, history_idx=%lu, preserved ADC history", 
         ULP_GET_U32(ulp_cycle_count), ULP_GET_U32(ulp_history_idx));
    
    /* Resume the program from entry point - ULP will check curr_wake_status,
     * see it's zero, and continue normal monitoring loop with existing history */
    uint32_t cycle_before = ULP_GET_U32(ulp_cycle_count);
    DLOG(TAG, "Calling ulp_run() with 125ms timer - cycle_count before=%lu", cycle_before);
    
    esp_err_t err = ulp_run((uint32_t*)&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        ELOG(TAG, "Failed to resume ULP program: %s", esp_err_to_name(err));
        return;
    }
    
    /* Give ULP time to start and increment cycle_count (one cycle = ~125ms, but may start immediately) */
    vTaskDelay(pdMS_TO_TICKS(200));
    uint32_t cycle_after = ULP_GET_U32(ulp_cycle_count);
    
    ILOG(TAG, "ULP program resumed: cycle_count before=%lu, after_200ms=%lu (delta=%ld)", 
         cycle_before, cycle_after, (int32_t)(cycle_after - cycle_before));
    
    if (cycle_after == cycle_before) {
        WLOG(TAG, "WARNING: ULP cycle_count did NOT increment after ulp_run() - ULP may not be running!");
    }
    
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    debug_ulp_status();
#endif
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
           (adc_get_ulp_button_wake_reason() == ULP_BUTTON_WAKE_LONG_PRESS);
}

// Check if same as last ADC wake reason (for suppression)
bool adc_ulp_same_adc_wake_reason(void) {
    return (adc_get_ulp_wake_reason() != ULP_ADC_WAKE_NONE) &&
           (adc_get_ulp_wake_reason() == adc_get_ulp_last_wake_reason());
}

void adc_ulp_clear_wake_sources(void) {
    FUNC_ENTRY(TAG);
    ULP_SET_U32(ulp_curr_wake_status, 0);
    // Don't clear last_wake_status here - it's updated on wake based on source

    /* Consume/clear any snapshot written by ULP - CPU is reading wake reason now */
    ULP_SET_U32(ulp_snapshot_valid, 0);
    ULP_SET_U32(ulp_snapshot_mad, 0);
    ULP_SET_U32(ulp_snapshot_state, 0);
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
    adc_ulp_uninit_pins();
    return wake_source;
}

bool ulp_history_snapshot_take(ulp_history_snapshot_t *out, bool compute_mad, int max_retries)
{
    if (!out) return false;
    FUNC_ENTRY_ARGSD(TAG, "Taking ULP history snapshot (compute_mad=%d, max_retries=%d)", compute_mad, max_retries);
    /* First, attempt to consume a frozen snapshot that the ULP wrote at wake time.
     * This provides the exact ULP state that triggered the wake. If present,
     * populate `out` from the snapshot and compute MAD over the preserved history
     * if requested. If no snapshot is present, fall back to validated live reads
     * using the existing retry logic. */
    uint32_t s_running = 0, s_idx = 0, s_cycles = 0, s_last = 0, s_mad = 0, s_state = 0;
    if (ulp_snapshot_read_and_consume_full(&s_running, &s_idx, &s_cycles, &s_last, &s_mad, &s_state)) {
        DLOG(TAG, "ULP snapshot consumed successfully");
        uint32_t valid = (s_cycles < ULP_ADC_HISTORY_SIZE) ? s_cycles : ULP_ADC_HISTORY_SIZE;
        if (valid == 0) {
            out->valid_count = 0;
            out->has_mad = false;
            return false;
        }
        out->running_sum = s_running;
        out->history_idx = s_idx;
        out->cycle_count = s_cycles;
        out->valid_count = valid;
        if (valid == ULP_ADC_HISTORY_SIZE) {
            out->history_avg = s_running >> ULP_ADC_HISTORY_SHIFT;
        } else {
            out->history_avg = s_running / valid;
        }
        out->last_sample = s_last & 0xFFF;
        /* Snapshot provides ULP-computed MAD/state if ULP was updated to write them.
         * Prefer ULP's values when available; otherwise compute MAD on CPU if requested. */
        out->snapshot_state = s_state;
        out->has_snapshot_state = true; /* snapshot was present and consumed */
        if (s_mad != 0) {
            out->mad = s_mad;
            out->has_mad = true;
        } else if (compute_mad) {
            uint32_t oldest = (s_idx + ULP_ADC_HISTORY_SIZE - valid) % ULP_ADC_HISTORY_SIZE;
            uint32_t mad = 0;
            for (uint32_t i = 0; i < valid; ++i) {
                uint32_t v = ULP_GET_ARR_U32(ulp_history, (oldest + i) % ULP_ADC_HISTORY_SIZE) & 0xFFF;
                mad += (v > out->history_avg) ? (v - out->history_avg) : (out->history_avg - v);
            }
            out->mad = mad / valid;
            out->has_mad = true;
        } else {
            out->has_mad = false;
        }
        return true;
    }

    /* No frozen snapshot — perform validated live reads with retry to avoid
     * cycle_count races when ULP is running. */
    int tries = 0;
    uint32_t before_cycle, after_cycle;
    uint32_t running_sum = 0;
    uint32_t history_idx = 0;

    do {
        before_cycle = ULP_GET_U32(ulp_cycle_count);
        running_sum = ULP_GET_U32(ulp_running_sum);
        history_idx = ULP_GET_U32(ulp_history_idx);
        after_cycle = ULP_GET_U32(ulp_cycle_count);
        tries++;
    } while ((before_cycle != after_cycle) && (tries < max_retries));

    uint32_t cycle_count = after_cycle;
    uint32_t valid = (cycle_count < ULP_ADC_HISTORY_SIZE) ? cycle_count : ULP_ADC_HISTORY_SIZE;
    if (valid == 0) {
        out->valid_count = 0;
        out->has_mad = false;
        return false;
    }

    out->running_sum = running_sum;
    out->history_idx = history_idx;
    out->cycle_count = cycle_count;
    out->valid_count = valid;

    if (valid == ULP_ADC_HISTORY_SIZE) {
        out->history_avg = running_sum >> ULP_ADC_HISTORY_SHIFT;
    } else {
        out->history_avg = running_sum / valid;
    }

    uint32_t last_idx = (history_idx == 0) ? (ULP_ADC_HISTORY_SIZE - 1) : (history_idx - 1);
    out->last_sample = ULP_GET_ARR_U32(ulp_history, last_idx) & 0xFFF;

    if (compute_mad) {
        uint32_t oldest = (history_idx + ULP_ADC_HISTORY_SIZE - valid) % ULP_ADC_HISTORY_SIZE;
        uint32_t mad = 0;
        for (uint32_t i = 0; i < valid; ++i) {
            uint32_t v = ULP_GET_ARR_U32(ulp_history, (oldest + i) % ULP_ADC_HISTORY_SIZE) & 0xFFF;
            mad += (v > out->history_avg) ? (v - out->history_avg) : (out->history_avg - v);
        }
        out->mad = mad / valid;
        out->has_mad = true;
    } else {
        out->has_mad = false;
    }
    return true;
}

/**
 * Diagnostic function to debug ULP status
 */
void debug_ulp_status(void) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    if(!ulp_initialized) {
        return;
    }
    printf("=== ULP Diagnostic Status ===\n");
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    printf("ULP Button Press Counter addr=%p, value=%lu (0x%04lX)\n", 
           &ulp_button_press_counter, ulp_button_press_counter_get(), ulp_button_press_counter_get());
    printf("ULP Button Last Result addr=%p, value=%lu (0x%04lX)\n", 
           &ulp_button_last_result, ulp_button_last_result_get(), ulp_button_last_result_get());
    // printf("ULP Button vars as uint32: counter=0x%08lX, last=0x%08lX\n",
    //        ulp_button_press_counter, ulp_button_last_result);
#endif
    printf("ULP Thresholds: Low=%lu, Rapid Change=%d\n", 
            ULP_GET_U32(ulp_low_threshold), ADC_RAPID_CHANGE_THRESHOLD);
    printf("ULP Cycle Count: %lu\n", ULP_GET_U32(ulp_cycle_count));
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    printf("ULP Last Result: %lu\n", ULP_GET_U32(ulp_last_result));
    printf("ULP Running Sum: %lu\n", ULP_GET_U32(ulp_running_sum));
    printf("ULP Cumulative Change: %lu\n", ULP_GET_U32(ulp_cum_change));
    printf("ULP Prev Result Index: %lu\n", ULP_GET_U32(ulp_history_idx));
    
    /* Debug: Calculate what the average SHOULD be */
    uint32_t manual_sum = 0;
    for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        manual_sum += ULP_GET_ARR_U32(ulp_history, i);
    }
    uint32_t manual_avg = manual_sum / ULP_ADC_HISTORY_SIZE;
    uint32_t ulp_avg = ULP_GET_U32(ulp_running_sum) >> ULP_ADC_HISTORY_SHIFT;
    // printf("  Debug: Manual sum=%lu, avg=%lu | ULP avg=%lu (shift=%d)\n",
    //        manual_sum, manual_avg, ulp_avg, ULP_ADC_HISTORY_SHIFT);
    // printf("  Debug: Expected diff=|%lu-%lu|=%lu, ULP cum_change=%lu\n",
    //        ULP_GET_U32(ulp_last_result), manual_avg,
    //        ULP_GET_U32(ulp_last_result) > manual_avg ? 
    //            ULP_GET_U32(ulp_last_result) - manual_avg : 
    //            manual_avg - ULP_GET_U32(ulp_last_result),
    //        ULP_GET_U32(ulp_cum_change));
    
    // printf("ULP Prev Result (raw): [");
    // for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
    //     printf("0x%04lX", ULP_GET_ARR_U32(ulp_history, i));
    //     if (i < (ULP_ADC_HISTORY_SIZE - 1)) printf(", ");
    // }
    // printf("]\n");
    printf("ULP Prev Result: [");
    for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        printf("%lu", ULP_GET_ARR_U32(ulp_history, i));
        if (i < (ULP_ADC_HISTORY_SIZE - 1)) printf(", ");
    }
    printf("]\n");
#endif
    // Display wake status - hybrid approach (2 packed variables)
    uint32_t curr_status = ULP_GET_U32(ulp_curr_wake_status);
    uint32_t last_status = ULP_GET_U32(ulp_last_wake_status);
    
    uint8_t curr_source = (curr_status >> ULP_WAKE_CURRENT_SOURCE_SHIFT) & 0x3;
    uint8_t curr_adc = (curr_status >> ULP_WAKE_CURRENT_ADC_SHIFT) & 0x7;
    uint8_t curr_button = (curr_status >> ULP_WAKE_CURRENT_BUTTON_SHIFT) & 0x7;
    uint8_t last_source = (last_status >> ULP_WAKE_CURRENT_SOURCE_SHIFT) & 0x3;
    uint8_t last_adc = (last_status >> ULP_WAKE_CURRENT_ADC_SHIFT) & 0x7;
    uint8_t last_button = (last_status >> ULP_WAKE_CURRENT_BUTTON_SHIFT) & 0x7;
    
    printf("ULP Wake Status (hybrid: 2 packed vars, curr=0x%02lX, last=0x%02lX):\n", curr_status & 0xFF, last_status & 0xFF);
    printf("  Current: Source=%s, ADC_reason=%s, Button_reason=%s\n", 
           adc_ulp_wake_sources_str(curr_source), 
           adc_ulp_adc_wake_reasons_str(curr_adc), 
           adc_ulp_button_wake_reasons_str(curr_button));
    printf("  Last:    Source=%s, ADC_reason=%s, Button_reason=%s\n", 
           adc_ulp_wake_sources_str(last_source), 
           adc_ulp_adc_wake_reasons_str(last_adc), 
           adc_ulp_button_wake_reasons_str(last_button));
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    printf("  Debug:   Button press in progress: counter=%lu (0=no press, >0=pressing, threshold=%d)\n", 
           ulp_button_press_counter_get(), ULP_LONG_PRESS_CYCLES);
#endif
    printf("=== End ULP Diagnostic ===\n");
#endif
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
    // Get ULP variables (raw ADC values, not voltage) - 32-bit word access with masking
    const uint32_t low_thr = ULP_GET_U32(ulp_low_threshold);         // Low threshold from config
    // const uint32_t high_thr = ADC_HIGH_THRESHOLD;    // High threshold from config
    const uint32_t rapid_thr = ADC_RAPID_CHANGE_THRESHOLD;  // Rapid change threshold
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    /* Unified snapshot reader: prefer frozen ULP snapshot (exact trigger state)
     * or validated live reads through ulp_history_snapshot_take(). */
    adc_snapshot_t snap = {0};
    bool have_snapshot = adc_snapshot_take(&snap, false, 3);
    uint32_t current_result = have_snapshot ? snap.last_sample : (ULP_GET_U32(ulp_last_result) & 0xFFF);

    uint32_t history_avg = 0; // Previous reading average
    uint32_t valid_count = 0;
    if (have_snapshot) {
        history_avg = snap.history_avg;
        valid_count = snap.valid_count;
    } else {
        valid_count = 0;
    }
    const int32_t change = (valid_count == 0) ? 0 : (int32_t)current_result - (int32_t)history_avg;
#else
    const uint32_t current_result = ULP_GET_U32(ulp_last_result) & 0xFFF;   // 12-bit ADC result (keep mask)
    const int32_t change = 0;
#endif
    DLOG(TAG, "ULP state analysis: current=%lu, "
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    "change=%ld, "
#endif
    "low_thr=%lu, rapid_thr=%lu",
         current_result,
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
         change, 
#endif
         low_thr, rapid_thr);
    
    // Check for low battery condition (primary ULP function)
    if (current_result <= low_thr) {
        WLOG(TAG, "ULP: detected low battery.");
        return ADC_BATTERY_LOW;
    }
    
    // Check for high battery condition (battery full detection)
    // if (current_result >= high_thr) {
    //     ILOG(TAG, "ULP: detected high battery.");
    //     return ADC_BATTERY_HIGH;
    // }
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)    
    // Check for rapid voltage changes (charging events detection)
    if (change != 0) {  // Only if we have a valid previous reading
        if ((change < -rapid_thr) || (change > rapid_thr)) {
            // Determine direction of change for charging detection
            if (change > 0) {  // Significant positive change
                ILOG(TAG, "ULP: Rapid increase suggests %s.", adc_battery_states_str(ADC_BATTERY_CHARGING_STARTED));
                return ADC_BATTERY_CHARGING_STARTED;
            } else if (change < 0) {  // Significant negative change
                ILOG(TAG, "ULP: Rapid decrease suggests %s.", adc_battery_states_str(ADC_BATTERY_CHARGING_STOPPED));
                return ADC_BATTERY_CHARGING_STOPPED;
            }
        }
    }
#endif
    // Default to normal state - ULP woke us but no specific condition detected
    ILOG(TAG, "ULP battery state: normal.");
    return ADC_BATTERY_NORMAL;
}

#endif /* CONFIG_ULP_COPROC_ENABLED */