/*
 * Unified ULP configuration header
 * Provides configuration values from Kconfig to ULP assembly program
 */

#ifndef D750C9AE_A16E_4110_820C_2D478C4A685F
#define D750C9AE_A16E_4110_820C_2D478C4A685F

#ifndef ULP_SENSORS_CONFIG_H
#define ULP_SENSORS_CONFIG_H

#include "sdkconfig.h"

/* ADC Configuration - Use definitions from adc_config.h */
#include "adc_config.h"

/* Button Configuration */
#ifdef CONFIG_ULP_BUTTON_ENABLED
#define ULP_BUTTON_GPIO        CONFIG_ULP_BUTTON_GPIO
#define ULP_BUTTON_RTC_IO      CONFIG_ULP_BUTTON_RTC_IO
#define ULP_BUTTON_LONG_PRESS_MS CONFIG_ULP_BUTTON_LONG_PRESS_MS
#else
#define ULP_BUTTON_GPIO        0
#define ULP_BUTTON_RTC_IO      11
#define ULP_BUTTON_LONG_PRESS_MS 3000
#endif

/* Timing Configuration */
#define ULP_CYCLE_TIME_MS      CONFIG_ULP_CYCLE_TIME_MS

/* Calculate thresholds based on timing */
#define ULP_CYCLES_PER_SECOND  (1000 / ULP_CYCLE_TIME_MS)
#define ULP_LONG_PRESS_CYCLES  ((ULP_BUTTON_LONG_PRESS_MS * ULP_CYCLES_PER_SECOND) / 1000)

/* Wake source bit definitions */
#define ULP_WAKE_SOURCE_ADC    (1 << 0)
#define ULP_WAKE_SOURCE_BUTTON (1 << 1)

/* ADC wake reasons */
#define ULP_ADC_WAKE_LOW_THR    1
#define ULP_ADC_WAKE_HIGH_THR   2  
#define ULP_ADC_WAKE_RAPID_CHG  3

/* Button wake reasons */
#define ULP_BUTTON_WAKE_LONG_PRESS 1

#endif /* ULP_SENSORS_CONFIG_H */


#endif /* D750C9AE_A16E_4110_820C_2D478C4A685F */
