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
#define RESULT_SIZE 6

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
#include "common_log.h"

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */
