#include "adc_private.h"

#if defined(CONFIG_ULP_COPROC_ENABLED)
#include "esp_system.h"
#include "ulp.h"
#include "soc/soc.h"

#include "ulp_program.h"
#include "adc_snapshot.h"
#include "ulp_adc_config.h"
#include "adc_ulp.h"

/* ULP binary references */
extern const uint8_t ulp_battery_bin_start[] asm("_binary_ulp_battery_bin_start");
extern const uint8_t ulp_battery_bin_end[]   asm("_binary_ulp_battery_bin_end");

/* ULP memory symbols */
extern uint32_t ulp_entry;

/* Forward declarations for functions defined elsewhere */

static const char *TAG = "ulp_prog";
RTC_DATA_ATTR bool ulp_prog_initialized = false;
RTC_DATA_ATTR bool ulp_prog_running = false;
enum reset_mode_e {
    RESET_MODE_ON_RESUME = 0,
    RESET_MODE_ON_START = 1,
    RESET_MODE_ON_INIT = 2
};

static void reset_ulp_vars(enum reset_mode_e mode) {
    FUNC_ENTRY(TAG);
    if(mode >= RESET_MODE_ON_START) {
        if(mode == RESET_MODE_ON_START) {
            DLOG(TAG, "Preserved last_wake_status=0x%02lX for next wake comparison", ULP_GET_U32(ulp_last_battery_state));
        }
        else if (mode == RESET_MODE_ON_INIT) { // init state
#if defined(CONFIG_ULP_BUTTON_ENABLED)
            ulp_button_press_counter = 0;
            ulp_curr_button_state = 0;
            ulp_last_button_state = 0;
#endif
            /* Full reset - also clear history and running sum */
            ulp_curr_battery_state = 0;
            ulp_last_battery_state = 0;
            ulp_plateau_direction = 0;
            ulp_plateau_count = 0;
            ulp_plateau_last_sample = 0;
            ulp_plateau_delta = 0;
            ulp_plateau_delta_sum = 0;
            ulp_plateau_delta_avg = 0;
            ulp_plateau_processing = 0;
            ulp_delta_abs = 0;
#if defined(DEBUG_ULP_VALUES)
            ulp_debug_adp_delta_path = 0;
            ulp_debug_delta_min = 0;
#endif
            ulp_last_result = 0;
            ulp_cycle_count = 0;
            ulp_main_cpu_running = 0;
#if defined(SNAPSHORT_AS_ARRAY)
            memset(&ulp_snapshot, 0, ULP_SNAPSHOT_RAW_FIELDS_NUM * sizeof(uint32_t));
#else
            ulp_snapshot_voltage = 0;
            ulp_snapshot_timestamp = 0;
            ulp_snapshot_valid = 0;
#endif

            memset(&ulp_slow_samples, 0, ULP_ADC_HISTORY_SIZE * sizeof(uint32_t));
            ulp_slow_idx = 0;
            ulp_slow_sum = 0;
            ulp_slow_avg = 0;
        }
    }
    // Always clear current battery state
}

/**
 * Initialize ULP program - load binary once at power-up
 * This runs only once when powered up (ulp_prog_initialized flag prevents reload)
 * IMPORTANT: On power-on reset, we must reload the ULP binary because RTC fast
 * memory is cleared. The ulp_prog_initialized flag persists in RTC slow memory, so we
 * check reset reason to detect true power-on and force reload.
 */
esp_err_t init_ulp_program(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;

    /* Check reset reason - on power-on reset, RTC fast memory (ULP code) is cleared
     * but RTC slow memory (ulp_prog_initialized flag) persists. We must reload binary. */
    esp_reset_reason_t reset_reason = esp_reset_reason();
    bool force_reload = false; // (reset_reason == ESP_RST_POWERON);

    if (force_reload) {
        ILOG(TAG, "Power-on reset detected - forcing ULP binary reload (ulp_prog_initialized=%d)", ulp_prog_initialized);
        ulp_prog_initialized = false;  // Clear flag to force reload
    }

    /* Check if ULP binary already loaded - run only once at power-up */
    if (ulp_prog_initialized) {
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

    compute_and_store_ulp_thresholds();

    reset_ulp_vars(RESET_MODE_ON_INIT);
    
    start_ulp_program();
    ulp_prog_initialized = true;
    ILOG(TAG, "ULP program loaded (%u bytes, %u words), last_wake_status initialized to 0",
         ulp_prog_size_bytes, ulp_prog_size_words);
    DLOG(TAG, "Raw ULP variable check (first boot - binary loaded):");
    DLOG(TAG, "  Thresholds: low=%lu, rapid_change=%d",
           ULP_GET_U32(ulp_calibrated_voltage_3V2), ADC_RAPID_CHANGE_THRESHOLD);
    DLOG(TAG, "  last_result addr=%p, value=0x%08lX (%lu)",
            &ulp_last_result, ULP_GET_U32(ulp_last_result), ULP_GET_U32(ulp_last_result));
    return ESP_OK;
}

bool ulp_prog_is_initialized(void)
{
    return ulp_prog_initialized;
}

bool ulp_prog_main_cpu_is_running(void)
{
#ifdef ULP_MODE
    return (ULP_GET_U32(ulp_main_cpu_running) != 0);
#else
    return false;
#endif
}

void ulp_prog_set_main_cpu_running(bool running)
{
#ifdef ULP_MODE
    ULP_SET_U32(ulp_main_cpu_running, running ? 1 : 0);
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

    // ILOG(TAG, "Resuming ULP program (post-ULP-wake - preserving ADC history)...");

    ulp_prog_set_main_cpu_running(true);
    ulp_live_snap_init();

    if(ulp_prog_running) goto done;

    /* CRITICAL: Add delay to ensure RTC memory write is visible to ULP before restart.
     * Without this, ULP may start before curr_wake_status clear propagates, see non-zero
     * wake status, jump to adc_frozen, and halt forever. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Ensure ULP timer is configured for periodic wakeup during deep sleep.
     * This is critical - without it, ULP only runs while main CPU is awake! */
    ulp_set_wakeup_period(0, TO_K_UL(ULP_CYCLE_TIME_MS)); // e.g., ULP_CYCLE_TIME_MS

    uint32_t cycle_before = ULP_GET_U32(ulp_cycle_count);
    DLOG(TAG, "Calling ulp_run()...");
    esp_err_t err = ulp_run((uint32_t*)&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        ELOG(TAG, "Failed to resume ULP program: %s", esp_err_to_name(err));
        return;
    }

    /* Give ULP time to start and increment cycle_count (one cycle = ~125ms, but may start immediately) */
    vTaskDelay(pdMS_TO_TICKS(ULP_CYCLE_TIME_MS));
    uint32_t cycle_after = ULP_GET_U32(ulp_cycle_count);

    ILOG(TAG, "...done: cycle_count: before=%lu, after_%dms=%lu (delta=%ld)",
         cycle_before, ULP_CYCLE_TIME_MS, cycle_after, (int32_t)(cycle_after - cycle_before));

    if (cycle_after == cycle_before) {
        WLOG(TAG, "WARNING: ULP cycle_count did NOT increment after ulp_run() - ULP may not be running!");
    }
    ulp_prog_running = true;
    done:
    ulp_live_snap_get();
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    debug_ulp_status();
#endif
}

void start_ulp_program(void)
{
    FUNC_ENTRY(TAG);
    // ILOG(TAG, "Starting ULP program (fresh sleep - clearing ADC history)...");

    /* Note: init_ulp_program() is now called in wakeup_init() at boot, not here.
     * This ensures ULP binary is loaded before any sleep operations. */
    init_ulp_adc();      // Initialize ULP ADC hardware (with locking)
    vTaskDelay(pdMS_TO_TICKS(50));
    reset_ulp_vars(RESET_MODE_ON_START);
    resume_ulp_program();
}
#endif // CONFIG_LOGGER_ADC_MODE_ULP