#ifndef F478CF77_DF17_4B41_970F_52702B05EC89
#define F478CF77_DF17_4B41_970F_52702B05EC89

#include "ulp_hardware.h"

// #include "ulp_battery.h"
/* ULP snapshot symbols (defined in ULP RTC fast memory by adc.S)
 * The ULP build embeds these labels and the component tooling exposes them
 * to the main firmware with an "ulp_" prefix (e.g. running_sum -> ulp_running_sum).
 * Use the ulp_snapshot_* names here so the C linker sees the symbols. */
extern uint32_t ulp_curr_wake_status;
extern uint32_t ulp_last_wake_status;
extern uint32_t ulp_cycle_count;
extern uint32_t ulp_history_idx;
extern uint32_t ulp_last_result;
extern uint32_t ulp_running_sum;
extern uint32_t ulp_charging_active;
extern uint32_t ulp_adaptive_threshold;
extern uint32_t ulp_history[ULP_ADC_HISTORY_SIZE];  /* Each: only lower 12 bits used */
extern uint32_t ulp_cum_change;            /* Only lower 16 bits used */
extern uint32_t ulp_main_cpu_running;

/* Snapshot variables for main CPU decision confirmations */
extern uint32_t ulp_snapshot_confirmation_avg;
extern uint32_t ulp_snapshot_baseline_avg;

/* History snapshot variables for main CPU access */
extern uint32_t ulp_snapshot_running_sum;
extern uint32_t ulp_snapshot_history_idx;
extern uint32_t ulp_snapshot_cycle_count;
extern uint32_t ulp_snapshot_valid_count;
extern uint32_t ulp_snapshot_history_avg;
extern uint32_t ulp_snapshot_last_sample;
extern uint32_t ulp_snapshot_cum_change;
extern uint32_t ulp_snapshot_mad;
extern uint32_t ulp_snapshot_state;
extern uint32_t ulp_snapshot_valid;

/* Confirmation phase variables */
extern uint32_t ulp_detection_phase;
extern uint32_t ulp_confirmation_sum;
extern uint32_t ulp_confirmation_count;
extern uint32_t ulp_confirmation_avg;
extern uint32_t ulp_detection_direction;
#if defined(CONFIG_ULP_BUTTON_ENABLED)
extern uint32_t ulp_button_press_counter;
#endif
/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */
extern uint32_t ulp_curr_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_last_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_low_threshold;

/**
 * @brief ULP Program Manager
 *
 * This module manages the lifecycle of ULP programs, including binary loading,
 * program initialization, starting, and resuming after wake events.
 */

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

#endif /* F478CF77_DF17_4B41_970F_52702B05EC89 */
