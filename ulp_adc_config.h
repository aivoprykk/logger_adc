#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

/* This header contains only preprocessor definitions that are safe for both
 * C code and ULP assembly files. No C types or function declarations.
 */

#include "sdkconfig.h"

#define JOIN(x, y) JOIN_AGAIN(x, y)
#define JOIN_AGAIN(x, y) x ## y

#define ULP_ADC_HISTORY_SIZE 4 // Must be power of 2: 2, 4, or 8 (reduced to 4 to save ULP RAM)

#if (ULP_ADC_HISTORY_SIZE == 16)
#define ULP_ADC_HISTORY_SHIFT 4
#elif (ULP_ADC_HISTORY_SIZE == 8)
#define ULP_ADC_HISTORY_SHIFT 3
#elif (ULP_ADC_HISTORY_SIZE == 4)
#define ULP_ADC_HISTORY_SHIFT 2
#elif (ULP_ADC_HISTORY_SIZE == 2)
#define ULP_ADC_HISTORY_SHIFT 1
#else
#error "ULP_ADC_HISTORY_SIZE must be 2, 4, or 8"
#endif

#define ULP_ADC_OVERSAMPLING 2  // 4 samples
#define ULP_ADC_STABILIZATION_DELAY 5000

// Single 16-bit variable layout:
// Bit 0-1:   current_wake_source (ADC=01, BUTTON=10, BOTH=11)
// Bit 2-4:   current_adc_reason (low=001, high=010, rapid=011)
// Bit 5-7:   current_button_reason (long_press=001)
// Bit 8-9:   last_wake_source (same encoding as current)
// Bit 10-12: last_adc_reason (same encoding as current)
// Bit 13-15: last_button_reason (same encoding as current)

// Bit masks and shifts
#define ULP_WAKE_CURRENT_SOURCE_MASK    0x0003
#define ULP_WAKE_CURRENT_SOURCE_SHIFT   0
#define ULP_WAKE_CURRENT_ADC_MASK       0x001C
#define ULP_WAKE_CURRENT_ADC_SHIFT      2
#define ULP_WAKE_CURRENT_BUTTON_MASK    0x00E0
#define ULP_WAKE_CURRENT_BUTTON_SHIFT   5
#define ULP_WAKE_LAST_SOURCE_MASK       0x0300
#define ULP_WAKE_LAST_SOURCE_SHIFT      8
#define ULP_WAKE_LAST_ADC_MASK          0x1C00
#define ULP_WAKE_LAST_ADC_SHIFT         10
#define ULP_WAKE_LAST_BUTTON_MASK       0xE000
#define ULP_WAKE_LAST_BUTTON_SHIFT      13
// Debug: Button press counter (2 bits, values 0-3)
#define ULP_WAKE_DEBUG_BTN_COUNT_MASK   0x30000
#define ULP_WAKE_DEBUG_BTN_COUNT_SHIFT  16

// Wake sources (3 possible values)
#define ULP_WAKE_SOURCE_NONE    0x0
#define ULP_WAKE_SOURCE_ADC     0x1
#define ULP_WAKE_SOURCE_BUTTON  0x2
#define ULP_WAKE_SOURCE_BOTH    0x3

// ADC wake reasons (4 possible values)
#define ULP_ADC_WAKE_NONE       0x0
#define ULP_ADC_WAKE_LOW_THR    0x1
#define ULP_ADC_WAKE_HIGH_THR   0x2
#define ULP_ADC_WAKE_RAPID_CHG  0x3

// Button wake reasons (2 possible values)
#define ULP_BUTTON_WAKE_NONE    0x0
#define ULP_BUTTON_WAKE_LONG_PRESS 0x1

#ifdef CONFIG_ULP_BUTTON_ENABLED
#if defined(CONFIG_HAS_BOARD_LILYGO_EPAPER_T5)
#if CONFIG_ULP_BUTTON_GPIO == 2 || CONFIG_ULP_BUTTON_GPIO == 13 || CONFIG_ULP_BUTTON_GPIO == 14 || CONFIG_ULP_BUTTON_GPIO == 15
#error "ULP_BUTTON_GPIO cannot be SDCARD pin GPIO2, GPIO13, GPIO14, or GPIO15"
#endif
#endif
#define ULP_BUTTON_GPIO        CONFIG_ULP_BUTTON_GPIO
#define ULP_BUTTON_LONG_PRESS_MS CONFIG_ULP_BUTTON_LONG_PRESS_MS
#endif

/* Timing Configuration */
#define ULP_CYCLE_TIME_MS      CONFIG_ULP_CYCLE_TIME_MS

/* Calculate thresholds based on timing */
#define ULP_CYCLES_PER_SECOND  (1000 / ULP_CYCLE_TIME_MS)
#define ULP_LONG_PRESS_CYCLES  ((ULP_BUTTON_LONG_PRESS_MS * ULP_CYCLES_PER_SECOND) / 1000)

/* Set low and high thresholds, approx. 3.27V - 4.1V*/
// high is set to 3000 to avoid false triggering when fully charged, 4.2v is around 2360
// low is set to 1770 to wake up when battery below 3.20v
// TODO: 1800 is around 3.4v - can be triggered also to inform low battery
#define ADC_LOW_TRESHOLD    1770
#define ADC_HIGH_TRESHOLD   3000

/* Rapid change threshold - wake up if voltage changes by more than this amount
 * between consecutive measurements (in ADC units, ~100 = ~0.08V change) */
#define ADC_RAPID_CHANGE_TRESHOLD   120

#define VOLTAGE_MAX 4200UL
#define VOLTAGE_MIN 3200UL
#define DEFAULT_VREF 1114UL
#define HIGH_RESISTOR 100000UL
#define LOW_RESISTOR 100000UL

#define VOLTAGE_PERC_COEF(a) (float)(1.0f - (float)((VOLTAGE_MAX - (uint32_t)(a)) / (VOLTAGE_MAX - VOLTAGE_MIN)))
#define VOLTAGE_PERC(a) (100UL * VOLTAGE_PERC_COEF(a))
#define VOLTAGE_CONV(a) (float)((HIGH_RESISTOR + LOW_RESISTOR) / LOW_RESISTOR * ((uint32_t)(a) / 100UL))
#define VOLTAGE_CONV_12(a) (float)((a) * 3300UL / 4095UL)
#define VOLTAGE_U32_TO_V(a) ((float)(a) / 1000.0f)

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

