#ifndef F9ED57EE_1235_4561_9E7B_DF850AA18991
#define F9ED57EE_1235_4561_9E7B_DF850AA18991

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "adc_defs.h"

#if !defined(ULP_MODE) && defined(CONFIG_ULP_COPROC_ENABLED)
#define ULP_MODE
#endif
// Define DEBUG_ULP_VALUES if ADC log level is set to INFO or more
#if (CONFIG_LOGGER_ADC_LOG_LEVEL <= LOG_DEBUG_NUM)
#define DEBUG_ULP_VALUES 1
#endif

// Rise thresholds (charging/load removal)
// NOTE: These are tuned for ADP plateau detection
#define RISE_THRESH_MAX          200
#define RISE_THRESH_HIGH         100

#define RISE_THRESH_CRITICAL     50
#define RISE_THRESH_DISCHARGING  50
#define RISE_THRESH_NOMINAL      40
#define RISE_THRESH_CHARGING     30
#define RISE_THRESH_FULL         25

// Fall thresholds (discharging/load applied)
#define FALL_THRESH_CRITICAL     45
#define FALL_THRESH_DISCHARGING  45
#define FALL_THRESH_NOMINAL      30
#define FALL_THRESH_CHARGING     25
#define FALL_THRESH_FULL         18

// ADP parameters
#define PLATEAU_SAMPLES_CRITICAL  2
#define PLATEAU_SAMPLES_NOMINAL   3
#define PLATEAU_SAMPLES_CHARGING  4
#define PLATEAU_COUNT_MAX         10  // Prevent counter from growing indefinitely

#define DELTA_MIN_RISE_CRITICAL  30
#define DELTA_MIN_RISE_FULL      15
#define DELTA_MIN_FALL_CRITICAL  20
#define DELTA_MIN_FALL_FULL      10

#define ABSOLUTE_MIN_THRESHOLD   15

/* Packed status utilities - Combined battery and button status */
#define BATTERY_STATUS_SHIFT 0
#define BATTERY_STATUS_MASK 0xFF
#define BUTTON_STATUS_SHIFT 8
#define BUTTON_STATUS_MASK 0xFF00

#define MASK(x, y) ((x) & (y))
#define UNPACK(x, y, z)  (MASK(x, y)) >> (z)
// PACK_SIMPLE: Packs a value into a field by shifting it left by 'shift' and masking with 'mask'.
// Usage: PACK_SIMPLE(value, mask, shift) - packs 'value' into the field defined by 'mask' at position 'shift'.
#define PACK_SIMPLE(value, mask, shift) MASK(((uint16_t)(value) << (shift)), mask)
// PACK_STATUS_FIELD: Sets a field in an existing packed variable 'x' to a new value 'v' using mask 'y' and shift 'z'.
// Usage: PACK_STATUS_FIELD(x, v, y, z) - replaces the field in 'x' defined by 'y' at position 'z' with 'v'.
#define PACK_STATUS_FIELD(packed, value, mask, shift) ((((packed) & ~(mask)) | (PACK_SIMPLE(value, mask, shift))))
#define GET_BATTERY_STATUS(x) UNPACK(x,BATTERY_STATUS_MASK,BATTERY_STATUS_SHIFT)
#define SET_BATTERY_STATUS(x, v) PACK_STATUS_FIELD(x, v, BATTERY_STATUS_MASK,BATTERY_STATUS_SHIFT)
#define GET_BUTTON_STATUS(x) UNPACK(x,BUTTON_STATUS_MASK,BUTTON_STATUS_SHIFT)
#define SET_BUTTON_STATUS(x, v) PACK_STATUS_FIELD(x, v, BUTTON_STATUS_MASK,BUTTON_STATUS_SHIFT)

/* Button status constants */
#define ULP_WAKE_STATUS_BUTTON_LONG_PRESS   (ULP_BUTTON_WAKE_REASON_LONG_PRESS << BUTTON_STATUS_SHIFT)
#define ULP_WAKE_STATUS_BUTTON_PRESS        (ULP_BUTTON_WAKE_REASON_PRESS << BUTTON_STATUS_SHIFT)

// ulp_snapshot array indices
// #define ULP_SNAPSHOT_CHARGING_ACTIVE_IDX    2 /* charging_active */
#define ULP_SNAPSHOT_VOLTAGE_IDX            0 /* last_result */
#define ULP_SNAPSHOT_TOTAL_SAMPLES_IDX      1 /* cycle_count */
#define ULP_SNAPSHOT_DATA_SOURCE_IDX        2 /* data_source */
#define ULP_SNAPSHOT_VALID_IDX              3 /* valid - set 0 from ulp */

#define ULP_SNAPSHOT_RAW_FIELDS_NUM         4 /* total num fields in array */

#define ULP_SNAPSHOT_GET_U32(idx) ULP_GET_ARR_U32(ulp_snapshot, (idx))

// ADC update interval in milliseconds
#define ADC_UPDATE_INTERVAL_MS CONFIG_ADC_CYCLE_TIME_MS

#define ADC_MAX_RAW 4095U    // 12-bit resolution
#define HYSTERESIS_PERCENT 5 // percent of threshold for clearing (example)

// Lipo battery voltage divider constants
#define VOLTAGE_MAX 4200UL
#define VOLTAGE_MIN 3200UL
#define DEFAULT_VREF 1114UL
#define HIGH_RESISTOR 100000UL
#define LOW_RESISTOR 100000UL

#define VOLTAGE_CONV_ADC_TO_MV_UL(mv) ((uint32_t)((uint64_t)(mv) * (HIGH_RESISTOR + LOW_RESISTOR) / LOW_RESISTOR))
// #define VOLTAGE_CONV_ADC_TO_MV_UL(mv) ((HIGH_RESISTOR + LOW_RESISTOR) / (uint32_t)((uint64_t)LOW_RESISTOR * (uint64_t)(mv)))
#define VOLTAGE_CONV_MV_TO_ADC_ULL(mv) (uint32_t)(((uint64_t)mv * (uint64_t)LOW_RESISTOR) / (HIGH_RESISTOR + LOW_RESISTOR))
#define VOLTAGE_CONV_MV_TO_V(a) (float)((a) / 1000.0f) /* millivolts to volts, have to be divided by float!! */
#define VOLTAGE_CONV_V(a) (float)((a) * 3300UL / 4095UL)

#define VOLTAGE_PERC_COEF(a) (float)(1.0f - (float)((VOLTAGE_MAX - (uint32_t)(a)) / (VOLTAGE_MAX - VOLTAGE_MIN)))
#define VOLTAGE_PERC(a) (100UL * VOLTAGE_PERC_COEF(a))

/* Fallback voltage constants */
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
#define USB_VOLTAGE_MAX_MV 5500UL  // Maximum expected USB charging voltage
#define FALLBACK_VOLTAGE_MV 3800UL // Generic estimate for other boards
#else
#define USB_VOLTAGE_MAX_MV 4500UL  // Maximum expected voltage for T5
#define FALLBACK_VOLTAGE_MV 3700UL // Conservative estimate for LilyGO boards
#endif

/* Common voltage thresholds */
// All thresholds now in millivolts
#define CHARGING_VOLTAGE_THRESHOLD_MV 4300UL  // Voltage above this indicates charging
#define BATTERY_VOLTAGE_MIN_MV 2800UL         // Minimum realistic operating voltage

/* Battery state thresholds in millivolts */
#define BATTERY_CRITICAL_LOW_MV 3220UL     // Critical low battery threshold
#define BATTERY_LOW_MV 3400UL             // Low battery threshold
#define BATTERY_HIGH_MV 4150UL             // High battery threshold (charging/full)


#define ULP_ADC_OVERSAMPLING 2  // 4 samples
#define ULP_ADC_STABILIZATION_DELAY 5000
#define ULP_CONFIRMATION_READINGS 3  // Number of confirmation readings

/* Set low and high thresholds, approx. 3.27V - 4.1V*/
// high is set to 3000 to avoid false triggering when fully charged, 4.2v is around 2360
// low is set to 1795 to wake up when battery below 3.25v
// TODO: 1795 is around 3.4v - can be triggered also to inform low battery
#define ADC_LOW_THRESHOLD    1795
#define ADC_HIGH_THRESHOLD   3000

/* Rapid change threshold - wake up if voltage changes by more than this amount
 * between consecutive measurements (in ADC units, ~110 = ~0.08V change) */
#define ADC_RAPID_CHANGE_THRESHOLD   110

#ifdef __cplusplus
}
#endif
#endif /* F9ED57EE_1235_4561_9E7B_DF850AA18991 */
