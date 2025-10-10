#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "ulp_adc_config.h"

#include "adc.h"
#include "adc_events.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_LOGGER_ADC_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_LOGGER_ADC_LOG_LEVEL
#endif
#include "common_log.h"

/* 
 * Safe ULP variable access macros
 * The ULP compiler generates uint32_t symbols and our assembly uses 32-bit words
 * These macros handle the type conversion safely
 */
#define ULP_GET_U32(var) (var & UINT16_MAX)
#define ULP_SET_U32(var, val) ((var) = (val))
#define ULP_GET_U16(var) (*(volatile uint16_t*)&(var) & UINT16_MAX)
#define ULP_SET_U16(var, val) (*(volatile uint16_t*)&(var) = (val))
#define ULP_GET_U8(var) (*(volatile uint8_t*)&(var) & UINT8_MAX)  
#define ULP_SET_U8(var, val) (*(volatile uint8_t*)&(var) = (val))
#define ULP_GET_ARR_U32(arr, i) (((volatile uint32_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U32(arr, i, val) (((volatile uint32_t*)&(arr))[i] = (val))
#define ULP_GET_ARR_U16(arr, i) (((volatile uint16_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U16(arr, i, val) (((volatile uint16_t*)&(arr))[i] = (val))

/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */
extern uint32_t ulp_curr_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_last_wake_status;  /* Packed: bits 0-1=source, 2-4=adc, 5-7=button */
extern uint32_t ulp_cycle_count;
extern uint32_t ulp_last_result;     /* Only lower 12 bits used */
extern uint32_t ulp_entry;

#ifdef CONFIG_ULP_BUTTON_ENABLED
extern uint32_t ulp_button_press_counter;
extern uint32_t ulp_button_last_result;
/* ULP st instruction always writes 32-bit, so .word packing doesn't work */
/* Must use .long and access as uint32_t */
#define ulp_button_press_counter_get() (ulp_button_press_counter & 0xFFFF)
#define ulp_button_press_counter_set(val) (ulp_button_press_counter = (val) & 0xFFFF)
#define ulp_button_last_result_get() (ulp_button_last_result & 0x1)
#define ulp_button_last_result_set(val) (ulp_button_last_result = (val) & 0x1)
#endif
#ifdef CONFIG_ULP_BATTERY_MONITORING_ENABLED
extern uint32_t ulp_cum_change;            /* Only lower 16 bits used */
extern uint32_t ulp_history[ULP_ADC_HISTORY_SIZE];  /* Each: only lower 12 bits used */
extern uint32_t ulp_history_idx;       /* Only lower 16 bits used */
extern uint32_t ulp_running_sum;           /* Full 32 bits used */
#endif

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

#define NO_OF_SAMPLES 64
#define RESULT_SIZE 16

#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)

#define VOLTAGE_ROW_SIZE 1
#define READ_LEN 64
#define ADC_CONV_MODE  ADC_CONV_SINGLE_UNIT_1
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#endif

/* Voltage filtering and history constants */
#define VOLTAGE_HISTORY_SIZE 16   // Power of 2 for efficient bit masking
#define VOLTAGE_FILTER_ALPHA 0.3f  // Exponential moving average factor (0.1 = heavy smoothing, 0.9 = responsive)

/* Fallback voltage constants */
#define FALLBACK_VOLTAGE_LILYGO 3.7f  // Conservative estimate for LilyGO boards
#define FALLBACK_VOLTAGE_GENERIC 3.8f // Generic estimate for other boards

/* Common voltage thresholds */
#define CHARGING_VOLTAGE_THRESHOLD 4.3f  // Voltage above this indicates charging
#define CHARGING_RANGE_MIN 4.0f          // Minimum voltage for charging range
#define USB_VOLTAGE_MAX 5.5f             // Maximum expected USB charging voltage  
#define BATTERY_VOLTAGE_MAX_T5 4.5f      // Maximum expected voltage for T5
#define BATTERY_VOLTAGE_MIN 2.0f         // Minimum realistic operating voltage

/* Battery state thresholds in millivolts */
#define BATTERY_CRITICAL_LOW_MV 3200     // Critical low battery threshold
#define BATTERY_LOW_MV 3400              // Low battery threshold
#define BATTERY_LOW_HYSTERESIS_MV 3500   // Higher threshold to exit low state
#define BATTERY_HIGH_MV 4100             // High battery threshold (charging/full)
#define BATTERY_HIGH_HYSTERESIS_MV 4000  // Lower threshold to exit high state

/* Rapid change detection thresholds for charging */
#define RAPID_CHANGE_THRESHOLD_MV 120    // Voltage change indicating charging started
#define RAPID_DROP_THRESHOLD_MV 100      // Voltage drop indicating charging stopped
#define SUDDEN_JUMP_THRESHOLD_MV 400     // Large voltage jump (charger connect)
#define SUDDEN_DROP_THRESHOLD_MV 300     // Large voltage drop (charger disconnect)

bool adc_lock(int timeout);
void adc_unlock();

/**
 * Common voltage validation function for both ULP and regular ADC
 * Validates voltage readings against board-specific thresholds
 * @param voltage_mv Voltage in millivolts
 * @param source_name Source description for logging ("ULP" or "ADC")
 * @return true if voltage is valid, false if likely pin conflict or unrealistic
 */
bool validate_voltage_reading(uint32_t voltage_mv, const char* source_name);

/**
 * Common calibration function for both ULP and regular ADC readings
 * Applies hardware calibration if available, falls back to voltage conversion
 * @param raw_adc Raw ADC reading value
 * @return Calibrated voltage in millivolts, or 0 on error
 */
uint32_t calibrate_adc_raw(uint32_t raw_adc);

/**
 * Validate and clamp voltage readings with board-specific logic
 * @param voltage Voltage in volts
 * @param is_display_s3 True for T-Display S3 boards, false for T5/other boards
 * @return Validated voltage, clamped to fallback value if invalid
 */
float validate_and_clamp_voltage(float voltage, bool is_display_s3);

/**
 * Common battery state event posting function
 * Handles posting ESP events for battery state changes
 */
static void post_battery_state_event(adc_battery_state_t state, const char* source);

bool adc_ulp_same_adc_wake_reason(void);

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */

