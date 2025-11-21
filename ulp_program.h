#ifndef F478CF77_DF17_4B41_970F_52702B05EC89
#define F478CF77_DF17_4B41_970F_52702B05EC89

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "adc_private_defs.h"

#ifdef ULP_MODE
#include "stdint.h"

extern uint32_t ulp_curr_battery_state;
extern uint32_t ulp_last_battery_state;
#if defined(CONFIG_ULP_BUTTON_ENABLED)
extern uint32_t ulp_button_press_counter;
extern uint32_t ulp_button_last_result;
extern uint32_t ulp_curr_button_state;
extern uint32_t ulp_last_button_state;
#endif
// extern uint32_t ulp_charging_active;

extern uint32_t ulp_plateau_last_sample;
extern uint32_t ulp_plateau_direction;
extern uint32_t ulp_plateau_count;
extern uint32_t ulp_plateau_delta;
extern uint32_t ulp_plateau_delta_sum;
extern uint32_t ulp_plateau_delta_avg;
extern uint32_t ulp_plateau_adaptive_threshold;
extern uint32_t ulp_plateau_processing;
extern uint32_t ulp_delta_abs;
extern uint32_t ulp_plateau_samples_needed;
;

#if defined(DEBUG_ULP_VALUES)
extern uint32_t ulp_debug_adp_delta_path;
extern uint32_t ulp_debug_delta_min;
#endif

extern uint32_t ulp_slow_samples[ULP_ADC_HISTORY_SIZE];
extern uint32_t ulp_slow_idx;
extern uint32_t ulp_slow_sum;
extern uint32_t ulp_slow_avg;
extern uint32_t ulp_slow_count;

extern uint32_t ulp_last_result;
extern uint32_t ulp_cycle_count;
extern uint32_t ulp_main_cpu_running;
/* History snapshot variables for main CPU access */
#ifdef SNAPSHORT_AS_ARRAY
extern uint32_t ulp_snapshot[];  /* ULP snapshot array in RTC memory */
#else
extern uint32_t ulp_snapshot_voltage;
extern uint32_t ulp_snapshot_timestamp;
extern uint32_t ulp_snapshot_valid;
#endif

extern uint32_t ulp_calibrated_voltage_3V2;
extern uint32_t ulp_calibrated_voltage_3V6;
extern uint32_t ulp_calibrated_voltage_3V8;
extern uint32_t ulp_calibrated_voltage_4V0;
extern uint32_t ulp_calibrated_voltage_4V1;
extern uint32_t ulp_calibrated_voltage_4V2;

/**
 * @brief Initialize ULP program - load binary once at power-up
 *
 * This runs only once when powered up (ulp_prog_initialized flag prevents reload).
 * IMPORTANT: On power-on reset, we must reload the ULP binary because RTC fast
 * memory is cleared. The ulp_prog_initialized flag persists in RTC slow memory, so we
 * check reset reason to detect true power-on and force reload.
 *
 * @return ESP_OK on success, error code otherwise
 */
// exported to adc.h
// esp_err_t init_ulp_program(void);

/**
 * @brief Start ULP program with fresh state
 *
 * Initializes ULP with cleared ADC history for fresh sleep cycle.
 * Battery voltage changes during wake time, so ULP needs fresh measurements
 * to accurately detect charging events in the new sleep cycle.
 */
void start_ulp_program(void);

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
void resume_ulp_program(void);

#endif /* CONFIG_ULP_COPROC_ENABLED */

#ifdef __cplusplus
}
#endif
#endif /* F478CF77_DF17_4B41_970F_52702B05EC89 */
