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
    ADC_BATTERY_CRITICAL_LOW = 5      /* Battery level critically low */
} adc_battery_state_t;

/* Main ADC functions */
int adc_init();
int adc_deinit();
float volt_read();
uint8_t calc_bat_perc_v(float adc);
uint8_t adc_on_ac();

/* Smart ADC reading functions for shared pin scenarios */
bool validate_adc_reading(float voltage);         /* Check if voltage reading is plausible */
float get_safe_battery_voltage(void);             /* Get validated voltage with conflict handling */
bool is_usb_charging(void);                       /* Detect USB charging state (LilyGO boards) */
float get_battery_voltage_compensated(void);      /* Get battery voltage compensated for charging */

/* Optimized display functions */
float adc_raw_to_voltage(uint32_t raw_adc_value); /* Convert raw ADC to calibrated voltage */
void get_battery_voltage_for_display(uint32_t raw_adc_value, float fallback_voltage, float *voltage_out); /* Optimized for display updates - uses reference */

/* ULP (Ultra Low Power) coprocessor functions - integrated into adc.c */
#if defined(CONFIG_ULP_COPROC_ENABLED)
esp_err_t init_ulp_program(void);
esp_err_t configure_ulp_wakeup(void);
void handle_ulp_wakeup(void);

/* ULP status and measurement functions */
uint32_t get_ulp_adc_reading(void);
uint32_t get_ulp_adc_calibrated(void);      /* Get calibrated ULP reading in mV (no averaging - ULP is stable) */
adc_battery_state_t get_ulp_battery_state(void);
void debug_ulp_status(void);                /* Debug function to show ULP status */
esp_err_t manual_trigger_ulp_measurement(void); /* Manual ULP trigger for testing */
void test_ulp_functionality(void);          /* Comprehensive ULP test */
#endif /* CONFIG_ULP_COPROC_ENABLED */

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
