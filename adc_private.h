#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "adc_private_defs.h"

#include "adc.h"
#include "adc_events.h"

#include "esp_adc/adc_cali.h"

#if (defined(CONFIG_LOGGER_USE_GLOBAL_LOG_LEVEL) && CONFIG_LOGGER_GLOBAL_LOG_LEVEL < CONFIG_LOGGER_ADC_LOG_LEVEL)
#define C_LOG_LEVEL CONFIG_LOGGER_GLOBAL_LOG_LEVEL
#else
#define C_LOG_LEVEL CONFIG_LOGGER_ADC_LOG_LEVEL
#endif
#include "common_log.h"

// #define USE_REF_SNAPSHOT

#ifdef ULP_MODE
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
#endif

#if defined(OLD_ADC)

typedef struct adc_snapshot_s {
#ifndef USE_ADAPTIVE_DELTA_PLATEAU
    uint32_t running_sum;    // ULP running_sum word
    uint32_t history_avg;    // computed average (raw units)
    uint32_t cum_change;     // ulp_cum_change word
#endif
    uint32_t cycle_count;
    uint32_t valid_count;
    uint32_t history_idx;
    uint32_t last_sample;
    uint32_t mad;
    bool has_mad;
    uint32_t snapshot_state;
    bool has_snapshot_state;
} adc_snapshot_t;
typedef adc_snapshot_t ulp_history_snapshot_t;

#endif

typedef struct adc_context_s {
    SemaphoreHandle_t xMutex;
    SemaphoreHandle_t batMutex;
    adc_cali_handle_t cali_handle;
    esp_timer_handle_t low_bat_timer;
    esp_timer_handle_t adc_timer;
    TaskHandle_t worker_task;
    uint32_t suppression_start_time_ms;
    uint32_t low_bat_start_time_ms;
    void (*low_battery_callback)(void);
    bool lcd_charge_notification;
    bool events_suppressed;
    uint8_t do_calibration;
    bool (*should_filter_charge_events)(void);
} adc_context_t;

#define ADC_CONTEXT_DEFAULT {0}

extern adc_context_t adc_ctx;

bool adc_lock(int timeout);
void adc_unlock(void);

uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten);
void adc_calibration_deinit(void);

uint32_t calibrate_adc_raw(uint32_t raw_adc);
float validate_and_clamp_voltage_mv(uint32_t voltage_mv);

static void post_battery_state_event(adc_battery_state_t state, const char* source);

bool adc_ulp_same_adc_wake_reason(void);

uint32_t compute_ulp_history_mad(void);
esp_err_t raw_to_mv_wrapper(int raw, uint32_t *voltage_mv);
uint32_t find_raw_for_pin_mv(uint32_t pin_mv);

uint32_t read_battery_adc(void);
esp_err_t adc_oneshot_init(void);
esp_err_t adc_oneshot_deinit(void);
esp_err_t compute_and_store_thresholds(void);

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */

