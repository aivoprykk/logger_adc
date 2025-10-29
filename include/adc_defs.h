#ifndef DBF28C18_61AA_4DD0_AD12_D2F736989011
#define DBF28C18_61AA_4DD0_AD12_D2F736989011

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
#define ADC_ULP_BAT_STATE_NORMAL       0x0
#define ADC_ULP_BAT_STATE_LOW          0x1
#define ADC_ULP_BAT_STATE_HIGH         0x2
#define ADC_ULP_BAT_STATE_CHARGING_STARTED  0x3
#define ADC_ULP_BAT_STATE_CHARGING_STOPPED  0x4
#define ADC_ULP_BAT_STATE_CRITICAL_LOW 0x5

/* ADC wake reasons with explicit values */
#define ADC_ULP_BAT_STATES(l) \
    l(NORMAL, 0x0) /* Battery level within normal range */ \
    l(LOW, 0x1) /* Battery level below low threshold */ \
    l(HIGH, 0x2) /* Battery level above high threshold (full) */ \
    l(CHARGING_STARTED, 0x3) /* Charging just started (rapid voltage increase) */ \
    l(CHARGING_STOPPED, 0x4) /* Charging stopped (voltage plateau/drop) */ \
    l(CRITICAL_LOW, 0x5) /* Battery level critically low */ \
    l(CHARGE_STABILIZED, 0x6) /* Battery voltage stabilized after charging */

/********************************************************************
 * Button Wake Reason Values - Single Source of Truth
 ********************************************************************/
#define ULP_BUTTON_WAKE_REASON_NONE        0x0
#define ULP_BUTTON_WAKE_REASON_LONG_PRESS  0x1

/* Button wake reasons with explicit values */
#define ADC_ULP_BUTTON_WAKE_REASONS(l) \
    l(NONE, 0x0) \
    l(LONG_PRESS, 0x1)

/********************************************************************
 * Enum Generator Macros for C Code (used in adc.h)
 ********************************************************************/
#define ADC_ULP_WAKE_SOURCES_ENUM(l, m) WAKE_SOURCE_##l = m,
#define ADC_ULP_ADC_WAKE_REASONS_ENUM(l, m) ADC_ULP_BATTERY_##l = m,
#define ADC_ULP_BUTTON_WAKE_REASONS_ENUM(l, m) ULP_ULP_BUTTON_WAKE_##l = m,

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

#define ULP_CONFIRMATION_READINGS 3  // Number of confirmation readings

#endif /* DBF28C18_61AA_4DD0_AD12_D2F736989011 */


