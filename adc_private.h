#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adc_config.h"

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

#define NO_OF_SAMPLES 64
#define RESULT_SIZE 8

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

#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_LOGGER_ADC_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_LOGGER_ADC_LOG_LEVEL
#endif

/* ULP configuration constants */
#define ULP_BATTERY_LOW_THRESHOLD 3400   // mV
#define ULP_BATTERY_HIGH_THRESHOLD 4100  // mV
#define ULP_BATTERY_CRITICAL_THRESHOLD 3200 // mV
#define ULP_READ_PERIOD_MS 500          // 500 ms

/* ULP variable addresses in RTC slow memory */
#define ULP_ADC_READING_ADDR    0
#define ULP_BATTERY_STATE_ADDR  1
#define ULP_WAKE_FLAG_ADDR      2

/* Voltage filtering and history constants */
#define VOLTAGE_HISTORY_SIZE 10  // Increased for better averaging
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

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */
