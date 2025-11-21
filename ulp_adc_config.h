#ifndef D1C9BA26_5394_40FF_AB0C_51CFF9C4C45C
#define D1C9BA26_5394_40FF_AB0C_51CFF9C4C45C

#ifdef __cplusplus
extern "C" {
#endif

#include "adc_private_defs.h"  /* Import public wake source/reason values - single source of truth */

#if defined(CONFIG_ULP_COPROC_ENABLED)
/*
 * Algorithm selection flag
 * Define USE_ADAPTIVE_DELTA_PLATEAU to use adaptive delta + plateau detection
 * instead of the current rapid change + confirmation logic
 */
#ifdef CONFIG_ULP_ADAPTIVE_DELTA_PLATEAU
#define USE_ADAPTIVE_DELTA_PLATEAU 1
#endif

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

// #define ULP_ADP_ADAPTIVE_THRESHOLD 1

#endif /* CONFIG_ULP_COPROC_ENABLED */

/********************************************************************
 * ULP ADC Configuration
 * Using ULP-specific names to avoid conflicts with ESP-IDF enums
 ********************************************************************/

/* ADC Channel selection based on CONFIG_ADC_CHANNEL */
#ifdef CONFIG_ADC_CHANNEL
#define ULP_ADC_CHANNEL  CONFIG_ADC_CHANNEL  /* Use configured value as-is */
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
 * Legacy compatibility - for C code that may need ESP-IDF types
 * These are only used in C files, not in ULP assembly
 ********************************************************************/
#define ULP_BUTTON_RTC_IO  CONFIG_ULP_BUTTON_RTC_IO

#ifdef __cplusplus
}
#endif
#endif /* D1C9BA26_5394_40FF_AB0C_51CFF9C4C45C */
