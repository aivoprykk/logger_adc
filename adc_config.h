#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

/* This header contains only preprocessor definitions that are safe for both
 * C code and ULP assembly files. No C types or function declarations.
 */

#include "sdkconfig.h"

#define JOIN(x, y) JOIN_AGAIN(x, y)
#define JOIN_AGAIN(x, y) x ## y

/* Set low and high thresholds, approx. 1.35V - 1.75V*/
#define ADC_LOW_TRESHOLD    1500
#define ADC_HIGH_TRESHOLD   2000

#define VOLTAGE_MAX 4200
#define VOLTAGE_MIN 3200
#define DEFAULT_VREF 1114
#define HIGH_RESISTOR 100000L
#define LOW_RESISTOR 100000L

#define VOLTAGE_PERC_COEF(a) (1 - ((VOLTAGE_MAX - (a)) / (VOLTAGE_MAX - VOLTAGE_MIN)))
#define VOLTAGE_PERC(a) (100 * VOLTAGE_PERC_COEF(a))
#define VOLTAGE_CONV(a) ((HIGH_RESISTOR + LOW_RESISTOR) / LOW_RESISTOR * ((a) / 100) * 1000)
#define VOLTAGE_CONV_12(a) ((a) * 3300 / 4095)
#define VOLTAGE_U32_TO_V(a) ((a) / 1000000)

/********************************************************************
 * ULP ADC Configuration
 * Using ULP-specific names to avoid conflicts with ESP-IDF enums
 ********************************************************************/

/* ADC Channel selection based on CONFIG_ADC_CHANNEL */
#ifdef CONFIG_ADC_CHANNEL
#if CONFIG_ADC_CHANNEL == 0
#define ULP_ADC_CHANNEL  0
#elif CONFIG_ADC_CHANNEL == 1
#define ULP_ADC_CHANNEL  1
#elif CONFIG_ADC_CHANNEL == 2
#define ULP_ADC_CHANNEL  2
#elif CONFIG_ADC_CHANNEL == 3
#define ULP_ADC_CHANNEL  3
#elif CONFIG_ADC_CHANNEL == 4
#define ULP_ADC_CHANNEL  4
#elif CONFIG_ADC_CHANNEL == 5
#define ULP_ADC_CHANNEL  5
#elif CONFIG_ADC_CHANNEL == 6
#define ULP_ADC_CHANNEL  6
#elif CONFIG_ADC_CHANNEL == 7
#define ULP_ADC_CHANNEL  7
#elif CONFIG_ADC_CHANNEL == 8
#define ULP_ADC_CHANNEL  8
#elif CONFIG_ADC_CHANNEL == 9
#define ULP_ADC_CHANNEL  9
#else
#define ULP_ADC_CHANNEL  CONFIG_ADC_CHANNEL  /* Use configured value as-is */
#endif
#else
#define ULP_ADC_CHANNEL  0  /* Default to channel 0 if not configured */
#endif

/* ADC Unit selection based on CONFIG_ADC_UNIT */
#ifdef CONFIG_ADC_UNIT
#if (CONFIG_ADC_UNIT == 1 || CONFIG_ADC_UNIT == 2)
#define ULP_ADC_UNIT (CONFIG_ADC_UNIT - 1)
#else
#define ULP_ADC_UNIT 0  /* Invalid config, default to unit 1 */
#endif
#else
#define ULP_ADC_UNIT 0  /* Default to unit 1 if not configured */
#endif

/* ADC Attenuation selection based on CONFIG_ADC_ATTEN */
#ifdef CONFIG_ADC_ATTEN
#if (CONFIG_ADC_ATTEN > 0 && CONFIG_ADC_ATTEN <= 2) || CONFIG_ADC_ATTEN == 25
#define ULP_ADC_ATTEN 1  /* 2.5dB attenuation */
#elif CONFIG_ADC_ATTEN <= 6
#define ULP_ADC_ATTEN 2  /* 6dB attenuation */
#elif CONFIG_ADC_ATTEN <= 12
#define ULP_ADC_ATTEN 3  /* 11/12dB attenuation */
#else
#define ULP_ADC_ATTEN 0  /* 0dB attenuation for other values */
#endif
#else
#define ULP_ADC_ATTEN 0  /* Default to 0dB attenuation if not configured */
#endif

/* ADC Bitwidth - using 12-bit as default */
#define ULP_ADC_BITWIDTH 12

/********************************************************************
 * Configuration Debug - Preprocessor values resolution
 * These will be visible in preprocessed output for debugging
 ********************************************************************/
/*
 * Resolved configuration values:
 * CONFIG_ADC_CHANNEL = CONFIG_ADC_CHANNEL
 * CONFIG_ADC_UNIT = CONFIG_ADC_UNIT
 * CONFIG_ADC_ATTEN = CONFIG_ADC_ATTEN
 * ULP_ADC_CHANNEL = ULP_ADC_CHANNEL
 * ULP_ADC_UNIT = ULP_ADC_UNIT
 * ULP_ADC_ATTEN = ULP_ADC_ATTEN
 */
/********************************************************************
 * Legacy compatibility - for C code that may need ESP-IDF types
 * These are only used in C files, not in ULP assembly
 ********************************************************************/
#ifdef __ASSEMBLER__
/* Assembly code - use our ULP definitions directly */
#define _ADC_UNIT_0     ULP_ADC_UNIT
#define _ADC_ATTEN      ULP_ADC_ATTEN
#else
/* C code - map to ESP-IDF enum values */
#if ULP_ADC_UNIT == 0
#define _ADC_UNIT_0     ADC_UNIT_1
#elif ULP_ADC_UNIT == 1
#define _ADC_UNIT_0     ADC_UNIT_2
#else
#define _ADC_UNIT_0     ADC_UNIT_1
#endif

#if ULP_ADC_ATTEN == 0
#define _ADC_ATTEN      ADC_ATTEN_DB_0
#elif ULP_ADC_ATTEN == 1
#define _ADC_ATTEN      ADC_ATTEN_DB_2_5
#elif ULP_ADC_ATTEN == 2
#define _ADC_ATTEN      ADC_ATTEN_DB_6
#elif ULP_ADC_ATTEN == 3
#define _ADC_ATTEN      ADC_ATTEN_DB_12  /* Use DB_12 instead of deprecated DB_11 */
#else
#define _ADC_ATTEN      ADC_ATTEN_DB_0
#endif

/* Map channel from config to ESP-IDF enum */
#if ULP_ADC_CHANNEL == 0
#define _ADC_CHANNEL_0  ADC_CHANNEL_0
#elif ULP_ADC_CHANNEL == 1
#define _ADC_CHANNEL_0  ADC_CHANNEL_1
#elif ULP_ADC_CHANNEL == 2
#define _ADC_CHANNEL_0  ADC_CHANNEL_2
#elif ULP_ADC_CHANNEL == 3
#define _ADC_CHANNEL_0  ADC_CHANNEL_3
#elif ULP_ADC_CHANNEL == 4
#define _ADC_CHANNEL_0  ADC_CHANNEL_4
#elif ULP_ADC_CHANNEL == 5
#define _ADC_CHANNEL_0  ADC_CHANNEL_5
#elif ULP_ADC_CHANNEL == 6
#define _ADC_CHANNEL_0  ADC_CHANNEL_6
#elif ULP_ADC_CHANNEL == 7
#define _ADC_CHANNEL_0  ADC_CHANNEL_7
#elif ULP_ADC_CHANNEL == 8
#define _ADC_CHANNEL_0  ADC_CHANNEL_8
#elif ULP_ADC_CHANNEL == 9
#define _ADC_CHANNEL_0  ADC_CHANNEL_9
#else
#define _ADC_CHANNEL_0  ADC_CHANNEL_0
#endif

/* Map bitwidth to ESP-IDF enum */
#if ULP_ADC_BITWIDTH == 9
#define _ADC_BITWIDTH   ADC_BITWIDTH_9
#elif ULP_ADC_BITWIDTH == 10
#define _ADC_BITWIDTH   ADC_BITWIDTH_10
#elif ULP_ADC_BITWIDTH == 11
#define _ADC_BITWIDTH   ADC_BITWIDTH_11
#elif ULP_ADC_BITWIDTH == 12
#define _ADC_BITWIDTH   ADC_BITWIDTH_12
#else
#define _ADC_BITWIDTH   ADC_BITWIDTH_DEFAULT
#endif
#endif /* __ASSEMBLER__ */

#endif /* ADC_CONFIG_H */

