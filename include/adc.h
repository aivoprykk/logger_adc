#ifndef F591E13F_5F2C_4BDB_9203_BB5C407DF403
#define F591E13F_5F2C_4BDB_9203_BB5C407DF403

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ADC battery state enumeration - used for both ULP and regular ADC modes */
typedef enum {
    ADC_BATTERY_NORMAL = 0,           /* Battery level within normal range */
    ADC_BATTERY_LOW = 1,              /* Battery level below low threshold */
    ADC_BATTERY_HIGH = 2,             /* Battery level above high threshold (full) */
    ADC_BATTERY_CHARGING_STARTED = 3, /* Charging just started (rapid voltage increase) */
    ADC_BATTERY_CHARGING_STOPPED = 4, /* Charging stopped (voltage plateau/drop) */
    ADC_BATTERY_CRITICAL_LOW = 5,     /* Battery level critically low */
    ADC_BATTERY_CHARGE_STABILIZED = 6 /* Battery voltage stabilized after charging */
} adc_battery_state_t;

/* Main ADC functions */
int adc_init();
int adc_deinit();
float volt_read();
uint8_t calc_bat_perc_v(float adc);
uint8_t adc_on_ac();

// bool validate_adc_reading(float voltage);         /* Check if voltage reading is plausible */
// float get_safe_battery_voltage(void);             /* Get validated voltage with conflict handling */
// bool is_usb_charging(void);                       /* Detect USB charging state (LilyGO boards) */
// float get_battery_voltage_compensated(void);      /* Get battery voltage compensated for charging */
// adc_battery_state_t get_battery_state(void);
/* Optimized display functions */
float adc_raw_to_voltage(uint32_t raw_adc_value); /* Convert raw ADC to calibrated voltage */
void get_battery_voltage_for_display(uint32_t raw_adc_value, float fallback_voltage, float *voltage_out); /* Optimized for display updates - uses reference */
void adc_sync_initial_charging_state(bool charging);
adc_battery_state_t get_adc_state(void);
bool get_adc_charging_state(void);                   /* Get current charging state - single source of truth */
bool adc_check_and_clear_lcd_charge_flag(void);     /* Check and clear LCD charge notification flag */

/* Battery monitoring and safety functions - managed by ADC module */
bool adc_check_battery_level(float minimum_voltage);  /* Check if battery is above minimum - thread safe */
void adc_set_low_battery_callback(void (*callback)(void)); /* Set callback for low battery detection */
void adc_set_minimum_battery_voltage(float voltage); /* Set minimum battery voltage threshold */

/* ADC event suppression functions - prevent false events during system transitions */
void adc_suppress_events(const char* reason);       /* Suppress ADC events with reason logging */
void adc_resume_events(const char* reason);         /* Resume ADC events with reason logging */
bool adc_should_suppress_event(int32_t event_id);   /* Check if specific event should be suppressed */

/* Forward declarations for app mode context - defined in main module */
typedef enum app_mode_s app_mode_t;

/* App mode context functions - implemented in main module for ADC charge state logic */
app_mode_t get_current_app_mode(void);              /* Get current app mode for charge state decisions */
bool get_current_charging_state(void);              /* Get current charging state for consistency */
bool should_filter_charge_events(void);             /* Check if charge events should be filtered */

#if defined(CONFIG_ULP_COPROC_ENABLED)

int init_ulp_program(void);
void start_ulp_program(void);
void debug_ulp_status(void);                /* Debug function to show ULP status */
adc_battery_state_t get_battery_state_from_ulp(void);  /* Get battery state using ULP variables on wakeup */
void adc_ulp_clear_last_wake_reason(void);
uint32_t adc_ulp_get_last_wake_reason(void);

/* Unified ULP sensor functions - for both ADC and button */
bool ulp_button_long_press_detected(void);             /* Check if ULP detected button long press */
bool ulp_adc_threshold_triggered(void);                /* Check if ULP detected ADC threshold trigger */
uint32_t ulp_get_adc_wake_reason(void);                /* Get specific ADC wake reason */
void ulp_clear_wake_sources(void);                     /* Clear all ULP wake sources */

#endif /* CONFIG_ULP_COPROC_ENABLED */

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
