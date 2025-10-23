#ifndef ADC_DEFS_H
#define ADC_DEFS_H

/* Public ADC definitions - safe for both C and assembly
 * This header contains ONLY preprocessor value definitions.
 * No C types, no function declarations - assembly-safe!
 * 
 * These are the single source of truth for wake source/reason values.
 * Both C code (via adc.h) and ULP assembly (via ulp_adc_config.h) use these.
 */

/********************************************************************
 * Wake Source Values - Single Source of Truth
 ********************************************************************/
#define ULP_WAKE_SOURCE_NONE    0x0
#define ULP_WAKE_SOURCE_ADC     0x1
#define ULP_WAKE_SOURCE_BUTTON  0x2
#define ULP_WAKE_SOURCE_BOTH    0x3

/* Wake sources with explicit values */
#define ADC_ULP_WAKE_SOURCES(l) \
    l(NONE, 0x0) \
    l(ADC, 0x1) \
    l(BUTTON, 0x2) \
    l(BOTH, 0x3)


/********************************************************************
 * ADC Wake Reason Values - Single Source of Truth
 ********************************************************************/
#define ULP_ADC_WAKE_NONE       0x0
#define ULP_ADC_WAKE_LOW_THR    0x1
#define ULP_ADC_WAKE_RAPID_CHG  0x2

/* ADC wake reasons with explicit values */
#define ADC_ULP_ADC_WAKE_REASONS(l) \
    l(NONE, 0x0) \
    l(LOW_THR, 0x1) \
    l(RAPID_CHG, 0x2)

/********************************************************************
 * Button Wake Reason Values - Single Source of Truth
 ********************************************************************/
#define ULP_BUTTON_WAKE_NONE        0x0
#define ULP_BUTTON_WAKE_LONG_PRESS  0x1

/* Button wake reasons with explicit values */
#define ADC_ULP_BUTTON_WAKE_REASONS(l) \
    l(NONE, 0x0) \
    l(LONG_PRESS, 0x1)

/********************************************************************
 * Enum Generator Macros for C Code (used in adc.h)
 ********************************************************************/
#define ADC_ULP_WAKE_SOURCES_ENUM(l, m) WAKE_SOURCE_##l = m,
#define ADC_ULP_ADC_WAKE_REASONS_ENUM(l, m) ULP_D_ADC_WAKE_##l = m,
#define ADC_ULP_BUTTON_WAKE_REASONS_ENUM(l, m) ULP_D_BUTTON_WAKE_##l = m,

#define ULP_ADC_HISTORY_SIZE 8 // Must be power of 2: 2, 4, or 8 (reduced to 4 to save ULP RAM)

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

#endif /* ADC_DEFS_H */


