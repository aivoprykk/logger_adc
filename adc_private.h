#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "ulp_adc_config.h"

#include "adc.h"
#include "adc_events.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "ulp_program.h"


#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_LOGGER_ADC_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_LOGGER_ADC_LOG_LEVEL
#endif
#include "common_log.h"

/* 
 * Safe ULP variable access macros
 * The ULP compiler generates uint32_t symbols and our assembly uses 32-bit words
 * These macros handle the type conversion safely
 */
#define ULP_GET_U32(var) (var & UINT16_MAX)
#define ULP_SET_U32(var, val) ((var) = (val))
#define ULP_GET_U16(var) (*(volatile uint16_t*)&(var) & UINT16_MAX)
#define ULP_SET_U16(var, val) (*(volatile uint16_t*)&(var) = (val))
#define ULP_GET_U8(var) (*(volatile uint8_t*)&(var) & UINT8_MAX)  
#define ULP_SET_U8(var, val) (*(volatile uint8_t*)&(var) = (val))
#define ULP_GET_ARR_U32(arr, i) (((volatile uint32_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U32(arr, i, val) (((volatile uint32_t*)&(arr))[i] = (val))
#define ULP_GET_ARR_U16(arr, i) (((volatile uint16_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U16(arr, i, val) (((volatile uint16_t*)&(arr))[i] = (val))

/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */
extern uint32_t ulp_cycle_count;
extern uint32_t ulp_last_result;     /* Only lower 12 bits used */

#ifdef CONFIG_ULP_BUTTON_ENABLED
extern uint32_t ulp_button_press_counter;
extern uint32_t ulp_button_last_result;
/* ULP st instruction always writes 32-bit, so .word packing doesn't work */
/* Must use .long and access as uint32_t */
#define ulp_button_press_counter_get() (ulp_button_press_counter & 0xFFFF)
#define ulp_button_press_counter_set(val) (ulp_button_press_counter = (val) & 0xFFFF)
#define ulp_button_last_result_get() (ulp_button_last_result & 0x1)
#define ulp_button_last_result_set(val) (ulp_button_last_result = (val) & 0x1)
#endif
#ifdef CONFIG_ULP_BATTERY_MONITORING_ENABLED
// extern uint32_t ulp_cum_change;            /* Only lower 16 bits used */
// extern uint32_t ulp_history[ULP_ADC_HISTORY_SIZE];  /* Each: only lower 12 bits used */
// extern uint32_t ulp_history_idx;       /* Only lower 16 bits used */
// extern uint32_t ulp_running_sum;           /* Full 32 bits used */
// extern uint32_t ulp_cum_change;     /* Full 32 bits used */
// extern uint32_t ulp_detection_phase;
// extern uint32_t ulp_detection_start_idx;
// extern uint32_t ulp_baseline_avg;
// extern uint32_t ulp_detection_direction;
// extern uint32_t ulp_confirmation_sum;

typedef struct {
    uint32_t running_sum;    // ULP running_sum word
    uint32_t history_idx;    // ulp_history_idx (next write index)
    uint32_t cycle_count;    // ulp_cycle_count
    uint32_t valid_count;    // min(cycle_count, ULP_ADC_HISTORY_SIZE)
    uint32_t history_avg;    // computed average (raw units)
    uint32_t last_sample;    // most recent sample in history
    uint32_t cum_change;     // ulp_cum_change word
    uint32_t mad;            // optional (computed only if requested)
#if defined(CONFIG_ULP_MAD_ENABLED)
    uint32_t snapshot_state; // optional snapshot state written by ULP
    uint16_t flags;  // bit 0: has_mad, bit 1: has_snapshot_state
    uint16_t reserved[3];
#else
    uint8_t flags;  // bit 0: has_mad, bit 1: has_snapshot_state
    uint8_t reserved[3];
#endif
} ulp_history_snapshot_t;

#if defined(CONFIG_ULP_MAD_ENABLED)
#define SNAPSHOT_HAS_MAD(snap) ((snap)->flags & 0x01)
#define SNAPSHOT_SET_MAD_FLAG(snap) do { (snap)->flags |= 0x01; } while(0)
#define SNAPSHOT_CLEAR_MAD_FLAG(snap) do { (snap)->flags &= ~0x01; } while(0)
#define SNAPSHOT_HAS_STATE(snap) ((snap)->flags & 0x02)
#define SNAPSHOT_SET_STATE(snap) do { (snap)->flags |= 0x02; } while(0)
#define SNAPSHOT_CLEAR_STATE(snap) do { (snap)->flags &= ~0x02; } while(0)
#else
#define SNAPSHOT_HAS_MAD(snap) ((snap)->flags != 0)
#define SNAPSHOT_SET_MAD_FLAG(snap) do { (snap)->flags = 1; } while(0)
#define SNAPSHOT_CLEAR_MAD_FLAG(snap) do { (snap)->flags = 0; } while(0)
#define SNAPSHOT_HAS_STATE(snap) ((snap)->flags != 0)
#define SNAPSHOT_SET_STATE(snap) do { (snap)->flags = 1; } while(0)
#define SNAPSHOT_CLEAR_STATE(snap) do { (snap)->flags = 0; } while(0)
#endif
/* Generic ADC snapshot type used by both ULP and non-ULP codepaths.
 * When ULP mode is enabled this is identical to ulp_history_snapshot_t.
 * Other modules should use `adc_snapshot_t` to get a unified view of
 * recent ADC history and optional MAD/state values. */
#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
typedef ulp_history_snapshot_t adc_snapshot_t;
#else
typedef struct adc_snapshot_s {
    uint32_t running_sum;
    uint32_t history_idx;
    uint32_t cycle_count;
    uint32_t valid_count;
    uint32_t history_avg;
    uint32_t last_sample;
    uint32_t mad;
    bool has_mad;
    uint32_t snapshot_state;
    bool has_snapshot_state;
} adc_snapshot_t;
#endif
#endif

#if !defined(CONFIG_LOGGER_ADC_MODE_ULP)
typedef struct {
    uint32_t result[RESULT_SIZE];
    uint8_t head;      // Next write position
    uint8_t count;
} adc_buffer_t;
#endif

typedef struct adc_context_s {
    uint8_t adc_initialized;
    uint8_t on_ac;
#if !defined(CONFIG_LOGGER_ADC_MODE_ULP)
    uint32_t adc_raw;
    uint32_t adc_voltage;
#endif
    uint8_t do_calibration;
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    uint32_t running_sum;
    uint32_t running_avg;
    uint32_t m_avg[3];
#endif
    SemaphoreHandle_t xMutex;
    SemaphoreHandle_t batMutex;
    adc_cali_handle_t cali_handle;
    esp_timer_handle_t low_bat_timer;
#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    esp_timer_handle_t adc_periodic_timer;
#endif
#if !defined(CONFIG_LOGGER_ADC_MODE_ULP)
    adc_buffer_t adc_buffer;
#endif
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_handle_t adc1_handle;
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_continuous_handle_t adc1_handle;
    TaskHandle_t adc_task_handle;
    uint32_t ret_num;
    uint8_t result[READ_LEN];
    uint8_t task_is_running;
#endif
} adc_context_t;

#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#define CTX_PART .adc_periodic_timer = NULL,
#else
#define CTX_PART .adc_task_handle = NULL, \
    .ret_num = 0, \
    .task_is_running = 1,
#endif

#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
#define AC_DET_PART .on_ac = 0, \
    .running_sum = 0, \
    .running_avg = 0, \
    .m_avg = {0},
#else
#define AC_DET_PART
#endif

#if !defined(CONFIG_LOGGER_ADC_MODE_ULP)
#define ADC_BUF .adc_buffer = {{0},0,0}, \
    .adc_raw = 0, \
    .adc_voltage = 0, \
    .adc1_handle = NULL,
#else
#define ADC_BUF
#endif

#define ADC_CONTEXT_DEFAULT { \
    .adc_initialized = 0, \
    .on_ac = 0, \
    .do_calibration = 0, \
    .xMutex = NULL, \
    .batMutex = NULL, \
    .cali_handle = NULL, \
    .low_bat_timer = NULL, \
    ADC_BUF \
    AC_DET_PART \
    CTX_PART \
}

extern adc_context_t adc_ctx;

#define ADC_MAX_RAW 4095U    // 12-bit resolution
#define HYSTERESIS_PERCENT 5 // percent of threshold for clearing (example)

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

#define NO_OF_SAMPLES 64
#define RESULT_SIZE 16

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

/* Voltage filtering and history constants */
// #define VOLTAGE_HISTORY_SIZE 16   // Power of 2 for efficient bit masking
// #define VOLTAGE_FILTER_ALPHA 0.3f  // Exponential moving average factor (0.1 = heavy smoothing, 0.9 = responsive)

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
#define USB_VOLTAGE_MAX_MV 5500UL             // Maximum expected USB charging voltage
#define FALLBACK_VOLTAGE_MV 3800UL // Generic estimate for other boards
#else
#define USB_VOLTAGE_MAX_MV 4500UL      // Maximum expected voltage for T5
#define FALLBACK_VOLTAGE_MV 3700UL  // Conservative estimate for LilyGO boards
#endif

/* Common voltage thresholds */
// All thresholds now in millivolts
#define CHARGING_VOLTAGE_THRESHOLD_MV 4300UL  // Voltage above this indicates charging
#define BATTERY_VOLTAGE_MIN_MV 2800UL         // Minimum realistic operating voltage

/* Battery state thresholds in millivolts */
#define BATTERY_CRITICAL_LOW_MV 3220UL     // Critical low battery threshold
#define BATTERY_LOW_MV 3400UL             // Low battery threshold
// #define BATTERY_LOW_HYSTERESIS_MV 3500UL   // Higher threshold to exit low state
// #define BATTERY_HIGH_HYSTERESIS_MV 4000UL  // Lower threshold to exit high state
#define BATTERY_HIGH_MV 4150UL             // High battery threshold (charging/full)

/* Rapid change detection thresholds for charging */
// #define RAPID_CHANGE_THRESHOLD_MV 120    // Voltage change indicating charging started
// #define RAPID_DROP_THRESHOLD_MV 100      // Voltage drop indicating charging stopped
// #define SUDDEN_JUMP_THRESHOLD_MV 400     // Large voltage jump (charger connect)
// #define SUDDEN_DROP_THRESHOLD_MV 300     // Large voltage drop (charger disconnect)

bool adc_lock(int timeout);
void adc_unlock(void);

uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten);
void adc_calibration_deinit(void);

/**
 * Common calibration function for both ULP and regular ADC readings
 * Applies hardware calibration if available, falls back to voltage conversion
 * @param raw_adc Raw ADC reading value
 * @return Calibrated voltage in millivolts, or 0 on error
 */
uint32_t calibrate_adc_raw(uint32_t raw_adc);

/**
 * Validate and clamp voltage readings with board-specific logic
 * @param voltage_mv Voltage in millivolts
 * @param is_display_s3 True for T-Display S3 boards, false for T5/other boards
 * @return Validated voltage, clamped to fallback value if invalid
 */
float validate_and_clamp_voltage_mv(uint32_t voltage_mv);

/**
 * Common voltage validation function for both ULP and regular ADC
 * Validates voltage readings against board-specific thresholds
 * @param voltage_mv Voltage in millivolts
 * @param source_name Source description for logging ("ULP" or "ADC")
 * @return true if voltage is valid, false if likely pin conflict or unrealistic
 */
// bool validate_voltage_reading(uint32_t voltage_mv);

/**
 * Common battery state event posting function
 * Handles posting ESP events for battery state changes
 */
static void post_battery_state_event(adc_battery_state_t state, const char* source);

bool adc_ulp_same_adc_wake_reason(void);

uint32_t compute_ulp_history_mad(void);

esp_err_t raw_to_mv_wrapper(int raw, uint32_t *voltage_mv);

#if defined(CONFIG_LOGGER_ADC_MODE_ULP)
/**
 * Read and validate ULP history aggregates into `out`.
 * - compute_mad: if true, compute exact MAD over valid_count samples (O(valid_count))
 * - max_retries: how many times to re-read if cycle_count changes (e.g., 2)
 * Returns true if snapshot has valid_count > 0, false otherwise.
 */
bool ulp_history_snapshot_take(ulp_history_snapshot_t *out, bool compute_mad, int max_retries);

/* Unified snapshot API for modules: will return recent history and optional MAD.
 * - compute_mad: compute MAD on CPU (if ULP didn't provide it)
 * - max_retries: how many times to try reading a consistent snapshot when ULP is running
 * Returns true if snapshot contains at least one valid sample. */
bool adc_snapshot_take(adc_snapshot_t *out, bool compute_mad, int max_retries);

void ulp_get_three_samples(uint32_t *current, uint32_t *prev1, uint32_t *prev2);

void compute_and_store_ulp_thresholds(uint32_t desired_batt_mv);
void debug_ulp_status(void);

// Snapshot accessor functions for confirmed values
uint32_t ulp_get_snapshot_confirmation_avg(void);
uint32_t ulp_get_snapshot_baseline_avg(void);

// History snapshot accessor functions
bool adc_ulp_is_snapshot_valid(void);
uint32_t adc_ulp_get_snapshot_running_sum(void);
uint32_t adc_ulp_get_snapshot_history_idx(void);
uint32_t adc_ulp_get_snapshot_cycle_count(void);
uint32_t adc_ulp_get_snapshot_valid_count(void);
uint32_t adc_ulp_get_snapshot_history_avg(void);
uint32_t adc_ulp_get_snapshot_last_sample(void);
uint32_t adc_ulp_get_snapshot_cum_change(void);
uint32_t adc_ulp_get_snapshot_mad(void);
uint32_t adc_ulp_get_snapshot_state(void);

// History snapshot control functions
bool adc_ulp_take_history_snapshot(uint32_t snapshot_state);

#endif

#define ADC_UPDATE_INTERVAL_MS CONFIG_ADC_CYCLE_TIME_MS

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */

