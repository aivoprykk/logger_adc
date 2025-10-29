#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

/* This header contains only preprocessor definitions that are safe for both
 * C code and ULP assembly files. No C types or function declarations.
 */

#include "sdkconfig.h"
#include "adc_defs.h"  /* Import public wake source/reason values - single source of truth */

/* Note: Wake source and reason values are now defined in adc_defs.h:
 * - ULP_WAKE_SOURCE_* (NONE, ADC, BUTTON, BOTH)
 * - ULP_ADC_WAKE_* (NONE, LOW_THR, RAPID_CHG)
 * - ULP_BUTTON_WAKE_* (NONE, LONG_PRESS)
 */

#define JOIN(x, y) JOIN_AGAIN(x, y)
#define JOIN_AGAIN(x, y) x ## y

// Hybrid 2-variable wake status layout (curr_wake_status & last_wake_status):
// Both variables use same bit layout (only CURRENT masks needed):
// Bit 0-1:   wake_source (ADC=01, BUTTON=10, BOTH=11)
// Bit 2-4:   adc_reason (none=000, low=001, rapid=010)
// Bit 5-7:   button_reason (none=000, long_press=001)

// Bit masks and shifts (CURRENT masks work for both curr and last variables)
#define ULP_WAKE_CURRENT_SOURCE_MASK    0x0003
#define ULP_WAKE_CURRENT_SOURCE_SHIFT   0
#define ULP_WAKE_CURRENT_ADC_MASK       0x001C
#define ULP_WAKE_CURRENT_ADC_SHIFT      2
#define ULP_WAKE_CURRENT_BUTTON_MASK    0x00E0
#define ULP_WAKE_CURRENT_BUTTON_SHIFT   5

/********************************************************************
 * Wake Status Values - Semi-Dynamic Generation Pattern
 * 
 * LIMITATION: The preprocessor cannot dynamically generate #define statements.
 * The # character is special and cannot be created by macro expansion.
 * 
 * SOLUTION: These #defines reference the base values from adc_defs.h container
 * macros (ULP_ADC_WAKE_*, ULP_BUTTON_WAKE_*), so changing values in ONE place
 * (adc_defs.h) automatically updates calculations here.
 * 
 * To add a new wake reason:
 * 1. Add to container macro in adc_defs.h: l(NEW_REASON, 0x3)
 * 2. Add ONE line here: #define ULP_WAKE_STATUS_ADC_NEW_REASON ((ULP_WAKE_SOURCE_ADC) | (ULP_ADC_WAKE_NEW_REASON << ULP_WAKE_CURRENT_ADC_SHIFT))
 * 
 * The value (0x3) only needs to be changed in adc_defs.h!
 ********************************************************************/

/* ADC wake status values: (source=ADC | (reason << ADC_SHIFT))
 * Values auto-calculated from ULP_ADC_WAKE_* constants in adc_defs.h */
#define ULP_BAT_STATUS_NORMAL        ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_NORMAL << ULP_WAKE_CURRENT_ADC_SHIFT))
#define ULP_BAT_STATUS_LOW     ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_LOW << ULP_WAKE_CURRENT_ADC_SHIFT))
#define ULP_BAT_STATUS_HIGH   ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_HIGH << ULP_WAKE_CURRENT_ADC_SHIFT))
#define ULP_BAT_STATUS_CHARGING_STARTED  ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_CHARGING_STARTED << ULP_WAKE_CURRENT_ADC_SHIFT))
#define ULP_BAT_STATUS_CHARGING_STOPPED  ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_CHARGING_STOPPED << ULP_WAKE_CURRENT_ADC_SHIFT))
#define ULP_BAT_STATUS_CRITICAL_LOW  ((ULP_WAKE_SOURCE_ADC) | (ADC_ULP_BAT_STATE_CRITICAL_LOW << ULP_WAKE_CURRENT_ADC_SHIFT))

/* Button wake status values: (source=BUTTON | (reason << BUTTON_SHIFT))
 * Values auto-calculated from ULP_BUTTON_WAKE_* constants in adc_defs.h */
#define ULP_WAKE_STATUS_BUTTON_NONE       ((ULP_WAKE_SOURCE_BUTTON) | (ULP_BUTTON_WAKE_REASON_NONE << ULP_WAKE_CURRENT_BUTTON_SHIFT))
#define ULP_WAKE_STATUS_BUTTON_LONG_PRESS ((ULP_WAKE_SOURCE_BUTTON) | (ULP_BUTTON_WAKE_REASON_LONG_PRESS << ULP_WAKE_CURRENT_BUTTON_SHIFT))

#ifdef CONFIG_ULP_BUTTON_ENABLED
#if defined(CONFIG_HAS_BOARD_LILYGO_EPAPER_T5)
#if CONFIG_ULP_BUTTON_GPIO == 2 || CONFIG_ULP_BUTTON_GPIO == 13 || CONFIG_ULP_BUTTON_GPIO == 14 || CONFIG_ULP_BUTTON_GPIO == 15
#error "ULP_BUTTON_GPIO cannot be SDCARD pin GPIO2, GPIO13, GPIO14, or GPIO15"
#endif
#endif
#define ULP_BUTTON_GPIO        CONFIG_ULP_BUTTON_GPIO
/* Button timing calculations - FIXED INTEGER MATH */
#define ULP_BUTTON_LONG_PRESS_MS  CONFIG_ULP_BUTTON_LONG_PRESS_MS
#endif

/* ULP timing calculations */
#define RTC_FAST_CLK_HZ        8000000UL    /* 8MHz RTC fast clock */
#define TICKS_PER_MS           (RTC_FAST_CLK_HZ / 1000UL)  /* 8000 ticks/ms */
#define MAX_WAIT_TICKS         65535UL      /* 16-bit limit for wait instruction */

#define ULP_CYCLE_TIME_MS      CONFIG_ADC_CYCLE_TIME_MS
#define TOTAL_TICKS_NEEDED     (ULP_CYCLE_TIME_MS * TICKS_PER_MS)

/* Calculate optimal wait parameters */
#if (TOTAL_TICKS_NEEDED <= MAX_WAIT_TICKS)
    #define ULP_WAIT_TICKS_PER_ITER  TOTAL_TICKS_NEEDED
    #define ULP_WAIT_ITERATIONS      1
#else
    #define ULP_WAIT_ITERATIONS      ((TOTAL_TICKS_NEEDED + MAX_WAIT_TICKS - 1) / MAX_WAIT_TICKS)
    #define ULP_WAIT_TICKS_PER_ITER  (TOTAL_TICKS_NEEDED / ULP_WAIT_ITERATIONS)
#endif

#ifdef CONFIG_ULP_BUTTON_ENABLED
// #define ULP_CYCLES_PER_SECOND  ((1000UL + ULP_CYCLE_TIME_MS - 1) / ULP_CYCLE_TIME_MS)  /* ceil(1000/cycle_ms) */
/* Calculate long press cycles with proper rounding */
/* round(ms * cycles/sec / 1000) */
// #define ULP_LONG_PRESS_CYCLES  ((ULP_BUTTON_LONG_PRESS_MS * ULP_CYCLES_PER_SECOND + 500) / 1000)
/* Alternative: Direct cycle calculation (more accurate) */
/* Direct calculation: cycles = ceil(press_time / cycle_time) */
#define ULP_LONG_PRESS_CYCLES  ((ULP_BUTTON_LONG_PRESS_MS + ULP_CYCLE_TIME_MS - 1) / ULP_CYCLE_TIME_MS)
#endif

/* Safety Checks */
#if (WAIT_TICKS_PER_ITER > MAX_WAIT_TICKS)
    #error "WAIT_TICKS_PER_ITER exceeds 16-bit limit"
#endif

#if (ULP_LONG_PRESS_CYCLES > 255)
    #error "ULP_LONG_PRESS_CYCLES too large for 8-bit counter"
#endif

/* Set low and high thresholds, approx. 3.27V - 4.1V*/
// high is set to 3000 to avoid false triggering when fully charged, 4.2v is around 2360
// low is set to 1795 to wake up when battery below 3.25v
// TODO: 1795 is around 3.4v - can be triggered also to inform low battery
#define ADC_LOW_THRESHOLD    1795
#define ADC_HIGH_THRESHOLD   3000

/* Rapid change threshold - wake up if voltage changes by more than this amount
 * between consecutive measurements (in ADC units, ~110 = ~0.08V change) */
#define ADC_RAPID_CHANGE_THRESHOLD   110

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
#define ULP_BUTTON_RTC_IO  CONFIG_ULP_BUTTON_RTC_IO
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

