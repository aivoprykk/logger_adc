#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define JOIN(x, y) JOIN_AGAIN(x, y)
#define JOIN_AGAIN(x, y) x ## y

#include "sdkconfig.h"

#define VOLTAGE_MAX 4195
#define VOLTAGE_MIN 3200
#define DEFAULT_VREF 1114
#define HIGH_RESISTOR 100000L
#define LOW_RESISTOR 100000L

#define VOLTAGE_PERC_COEF(a) (1 - ((VOLTAGE_MAX - (a)) / (VOLTAGE_MAX - VOLTAGE_MIN)))
#define VOLTAGE_PERC(a) (100 * VOLTAGE_PERC_COEF(a))
#define VOLTAGE_CONV(a) ((HIGH_RESISTOR + LOW_RESISTOR) / LOW_RESISTOR * ((a) / 100) * 1000)
#define VOLTAGE_CONV_12(a) ((a) * 3300 / 4095)
#define VOLTAGE_U32_TO_V(a) ((a) / 1000000)
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

/// ADC_UNIT definition
#if defined(CONFIG_ADC_UNIT) && (CONFIG_ADC_UNIT == 1 || CONFIG_ADC_UNIT == 2)
#define _ADC_UNIT_0 JOIN(ADC_UNIT_, CONFIG_ADC_UNIT)
#else
#define _ADC_UNIT_0 ADC_UNIT_1
#endif
/// ADC_ATTEN definition
#if defined(CONFIG_ADC_ATTEN)
#if (CONIG_ADC_ATTEN > 0 && CONIG_ADC_ATTEN <= 2) || CONFIG_ADC_ATTEN == 25
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 2_5)
#elif CONFIG_ADC_ATTEN <= 6
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 6)
#elif CONFIG_ADC_ATTEN <= 12
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 12)
#else
#define _ADC_ATTEN ADC_ATTEN_DB_0
#endif
#else
#if ESP_IDF_VERSION_MAJOR < 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR <= 1 && ESP_IDF_VERSION_PATCH < 3)
#define _ADC_ATTEN ADC_ATTEN_DB_11
#else
#define _ADC_ATTEN ADC_ATTEN_DB_12
#endif
#endif
/// ADC_CHANNEL definition
#if defined(CONFIG_ADC_CHANNEL)
#define _ADC_CHANNEL_0 JOIN(ADC_CHANNEL_, CONFIG_ADC_CHANNEL)
#else
#if CONFIG_IDF_TARGET_ESP32
#define _ADC_CHANNEL_0 ADC_CHANNEL_7
#if E_USE_ADC1_2
#define _ADC_CHANNEL_1 ADC_CHANNEL_5
#endif
#else
#define _ADC_CHANNEL_0 ADC_CHANNEL_3
#if E_USE_ADC1_2
#define _ADC_CHANNEL_1 ADC_CHANNEL_0
#endif
#endif
#endif
/// ADC_BITWIDTH definition
#if defined(CONFIG_ADC_BITWIDTH)
#if CONFIG_ADC_BITWIDTH == 0 || CONFIG_ADC_BITWIDTH < 9 || CONFIG_ADC_BITWIDTH > 12
#define _ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#else
// #define _ADC_BITWIDTH JOIN(ADC_WIDTH_BIT_, CONFIG_ADC_BITWIDTH)
#define _ADC_BITWIDTH JOIN(ADC_BITWIDTH_, CONFIG_ADC_BITWIDTH)
#endif
#else
#define _ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#endif

#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_LOGGER_ADC_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_LOGGER_ADC_LOG_LEVEL
#endif
#include "common_log.h"

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */
