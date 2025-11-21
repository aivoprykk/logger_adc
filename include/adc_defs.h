#ifndef DBF28C18_61AA_4DD0_AD12_D2F736989011
#define DBF28C18_61AA_4DD0_AD12_D2F736989011

#ifdef __cplusplus
extern "C" {
#endif

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
#define ULP_WAKE_SOURCE_BATTERY 0x1
#define ULP_WAKE_SOURCE_BUTTON  0x2

/* Wake sources with explicit values */
#define ADC_WAKE_SOURCES(l) \
    l(NONE) \
    l(BATTERY) \
    l(BUTTON)


#define ADC_BAT_STATES_ENUM(l) ADC_BATTERY_##l,
#define ADC_BUTTON_STATES_ENUM(l) ADC_BUTTON_##l,
#define ADC_ULP_WAKE_SOURCES_ENUM(l) WAKE_SOURCE_##l,
#define ADC_ULP_ADC_WAKE_REASONS_ENUM(l) ADC_ULP_BATTERY_##l,
#define ADC_ULP_BUTTON_WAKE_REASONS_ENUM(l) ULP_ULP_BUTTON_WAKE_##l,

/********************************************************************
 * ADC Wake Reason Values - Single Source of Truth
 ********************************************************************/
#define ADC_ULP_BAT_STATE_NORMAL       0x0
#define ADC_ULP_BAT_STATE_CRITICAL_LOW 0x1
#define ADC_ULP_BAT_STATE_CHARGING_STARTED  0x2
#define ADC_ULP_BAT_STATE_CHARGING_STOPPED  0x3

/* ADC wake reasons with explicit values */
#define ADC_BAT_STATES(l) \
    l(NORMAL) /* Battery level within normal range */ \
    l(CRITICAL_LOW) /* Battery level critically low */ \
    l(CHARGING) /* Charging just started (rapid voltage increase) */ \
    l(CHARGING_STOPPED) /* Charging stopped (voltage plateau/drop) */


/********************************************************************
 * Button Wake Reason Values - Single Source of Truth
 ********************************************************************/
#define ULP_BUTTON_WAKE_REASON_NONE        0x0
#define ULP_BUTTON_WAKE_REASON_PRESS       0x1
#define ULP_BUTTON_WAKE_REASON_LONG_PRESS  0x2

/* Button wake reasons with explicit values */
#define ADC_BUTTON_STATES(l) \
    l(NONE) \
    l(PRESS) \
    l(LONG_PRESS)

#define ULP_WAKE_SRC_SHIFT   8
#define ULP_WAKE_SRC_MASK    0xFF00
#define ULP_WAKE_STATUS_MASK 0x00FF

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

#ifdef __cplusplus
}
#endif

#endif /* DBF28C18_61AA_4DD0_AD12_D2F736989011 */


