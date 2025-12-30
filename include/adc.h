#ifndef F591E13F_5F2C_4BDB_9203_BB5C407DF403
#define F591E13F_5F2C_4BDB_9203_BB5C407DF403

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adc_defs.h"  /* Assembly-safe public definitions */

/* Note: ADC_ULP_ADC_WAKE_REASONS, ADC_ULP_BUTTON_WAKE_REASONS, and ADC_ULP_WAKE_SOURCES
 * are now defined in adc_defs.h as the single source of truth */

/* ADC battery state enumeration - used for both ULP and regular ADC modes */
typedef enum {
    ADC_BAT_STATES(ADC_BAT_STATES_ENUM)
} adc_battery_state_t;

typedef enum {
    ADC_BUTTON_STATES(ADC_BUTTON_STATES_ENUM)
} adc_button_state_t;

/* ULP wake reason enumerations - public API */
typedef enum {
    ADC_BAT_STATES(ADC_ULP_ADC_WAKE_REASONS_ENUM)
} adc_ulp_adc_wake_reason_t;

typedef enum {
    ADC_BUTTON_STATES(ADC_ULP_BUTTON_WAKE_REASONS_ENUM)
} adc_ulp_button_wake_reason_t;

typedef enum {
    ADC_WAKE_SOURCES(ADC_ULP_WAKE_SOURCES_ENUM)
} adc_ulp_wake_source_t;

/* String arrays for enums (for debugging) */
const char* adc_battery_states_str(int i);
const char* adc_wake_sources_str(int i);
const char* adc_button_wake_reasons_str(int i);

/* Main ADC functions */
int adc_init();
int adc_deinit();
uint8_t adc_calc_bat_perc(float adc);
uint8_t adc_is_charging();

float adc_get_cached_batt_volt();
uint32_t adc_get_cached_batt_mv(void);

// bool validate_adc_reading(float voltage);         /* Check if voltage reading is plausible */
// float get_safe_battery_voltage(void);             /* Get validated voltage with conflict handling */
// bool is_usb_charging(void);                       /* Detect USB charging state (LilyGO boards) */
// float get_battery_voltage_compensated(void);      /* Get battery voltage compensated for charging */
// adc_battery_state_t get_battery_state(void);
/* Optimized display functions */
void get_battery_voltage_for_display(float *voltage_out); /* Optimized for display updates - uses reference */
// void adc_sync_initial_charging_state(bool charging);
uint8_t battery_get_current_battery_state(void);
uint8_t battery_get_last_battery_state(void);
uint8_t battery_get_pending_battery_event(void);
#if defined(CONFIG_ULP_BUTTON_ENABLED)
uint8_t battery_get_current_button_state(void);
uint8_t battery_get_last_button_state(void);
uint8_t battery_get_pending_button_event(void);
#endif
bool adc_check_and_clear_lcd_charge_flag(void);     /* Check and clear LCD charge notification flag */

/* Battery monitoring and safety functions - managed by ADC module */
bool adc_check_battery_level(void);  /* Check if battery is above minimum - thread safe */
void adc_set_low_battery_callback(void (*callback)(void)); /* Set callback for low battery detection */
// void adc_set_minimum_battery_voltage(float voltage); /* Set minimum battery voltage threshold */

/* ADC event suppression functions - prevent false events during system transitions */
void adc_suppress_events(const char* reason);       /* Suppress ADC events with reason logging */
void adc_resume_events(const char* reason);         /* Resume ADC events with reason logging */
bool adc_should_suppress_event(int event_id);   /* Check if specific event should be suppressed */

/* Forward declarations for app mode context - defined in main module */
typedef enum app_mode_s app_mode_t;

/* App mode context functions - implemented in main module for ADC charge state logic */
// app_mode_t get_current_app_mode(void);              /* Get current app mode for charge state decisions */
// bool should_filter_charge_events(void);             /* Check if charge events should be filtered */

#if defined(CONFIG_ULP_COPROC_ENABLED)

int init_ulp_program(void);                        /* Load ULP binary (runs once at power-up) */
esp_err_t init_ulp_adc(void);                      /* Initialize ULP ADC hardware (with locking) */
void deinit_ulp_adc(void);                         /* Deinitialize ULP ADC hardware (with locking) */
void debug_ulp_status(void);                /* Debug function to show ULP status */
void debug_ulp_get_status(void);         /* Debug function to get ULP snapshot and calibration */

uint8_t get_battery_state_from_ulp(void);  /* Get battery state using ULP variables on wakeup */

void adc_ulp_clear_wake_sources(bool preserve_last_status);                     /* Clear all ULP wake sources */
int adc_ulp_after_wake(void);                /* Update last based on current wake source */
bool adc_ulp_threshold_triggered(void);                /* Check if ULP detected ADC threshold trigger */
uint32_t adc_ulp_get_cycle_count(void);                /* Get ULP cycle count - safe RTC memory access */

/* Unified ULP sensor functions - for both ADC and button */
bool adc_ulp_button_long_press_detected(void);             /* Check if ULP detected button long press */

/**
 * @brief Resume ULP program after ULP wake (preserves ADC history)
 * 
 * Resumes ULP execution without clearing ADC history. Used when going back to
 * sleep immediately after processing a ULP wake event. Preserves monitoring
 * state so rapid change detection continues to work.
 */
void resume_ulp_program(void);

void ulp_prog_set_main_cpu_running(bool running);
bool ulp_prog_main_cpu_is_running(void);
bool ulp_prog_is_initialized(void);

struct battery_snapshot_s* battery_update_snapshot(uint16_t adc_reading);

#endif /* CONFIG_ULP_COPROC_ENABLED */

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
