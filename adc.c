#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_ENABLED)

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#include <esp_idf_version.h>

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#include "esp_adc/adc_continuous.h"
#endif
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "adc.h"
#include "adc_events.h"

ESP_EVENT_DEFINE_BASE(ADC_EVENT);
#if (C_LOG_LEVEL < 2)
static const char * _adc_event_strings[] = { ADC_EVENT_LIST(STRINGIFY) };
const char * adc_event_strings(int id) {
    return _adc_event_strings[id];
}
#else
const char * adc_event_strings(int id) {return "ADC_EVENT";}
#endif

const static char *TAG = "adc";

/// ADC_UNIT definition
#if defined(CONFIG_ADC_UNIT) && (CONFIG_ADC_UNIT == 1 || CONFIG_ADC_UNIT == 2)
#define _ADC_UNIT_0 JOIN(ADC_UNIT_, CONFIG_ADC_UNIT)
#else
#define _ADC_UNIT_0 ADC_UNIT_1
#endif
/// ADC_ATTEN definition
#if defined(CONFIG_ADC_ATTEN)
#if (CONIG_ADC_ATTEN > 0 && CONIG_ADC_ATTEN <= 2) || CONFIG_ADC_ATTEN == 25
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 2_5)
#elif CONFIG_ADC_ATTEN <= 6
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 6)
#elif CONFIG_ADC_ATTEN <= 12
#define _ADC_ATTEN JOIN(ADC_ATTEN_DB_, 12)
#else
#define _ADC_ATTEN ADC_ATTEN_DB_0
#endif
#else
#if ESP_IDF_VERSION_MAJOR < 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR <= 1 && ESP_IDF_VERSION_PATCH < 3)
#define _ADC_ATTEN ADC_ATTEN_DB_11
#else
#define _ADC_ATTEN ADC_ATTEN_DB_12
#endif
#endif
/// ADC_CHANNEL definition
#if defined(CONFIG_ADC_CHANNEL)
#define _ADC_CHANNEL_0 JOIN(ADC_CHANNEL_, CONFIG_ADC_CHANNEL)
#else
#if CONFIG_IDF_TARGET_ESP32
#define _ADC_CHANNEL_0 ADC_CHANNEL_7
#if E_USE_ADC1_2
#define _ADC_CHANNEL_1 ADC_CHANNEL_5
#endif
#else
#define _ADC_CHANNEL_0 ADC_CHANNEL_3
#if E_USE_ADC1_2
#define _ADC_CHANNEL_1 ADC_CHANNEL_0
#endif
#endif
#endif
/// ADC_BITWIDTH definition
#if defined(CONFIG_ADC_BITWIDTH)
#if CONFIG_ADC_BITWIDTH == 0 || CONFIG_ADC_BITWIDTH < 9 || CONFIG_ADC_BITWIDTH > 12
#define _ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#else
#define _ADC_BITWIDTH JOIN(ADC_WIDTH_BIT_, CONFIG_ADC_BITWIDTH)
#endif
#else
#define _ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#endif

#define V_GRAPH_LIPO_LEN 21
#define ADJ_LENGTH 24

typedef struct adc_context_s {
    uint32_t adc_raw;
    uint32_t adc_voltage;
    uint8_t do_calibration;
    adc_cali_handle_t adc1_cali_handle;
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_handle_t adc1_handle;
    esp_timer_handle_t adc_periodic_timer;
    uint32_t result[RESULT_SIZE];
    int32_t result_index;
    SemaphoreHandle_t xMutex;
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_continuous_handle_t adc1_handle;
    TaskHandle_t adc_task_handle;
    uint32_t ret_num;
    uint8_t result[READ_LEN];
    uint8_t task_is_running;
#endif
} adc_context_t;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#define CTX_PART .adc_periodic_timer = NULL, \
    .result = {0}, \
    .result_index = -1, \
    .xMutex = NULL,
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#define CTX_PART .adc_task_handle = NULL, \
    .ret_num = 0, \
    .result = {0}, \
    .task_is_running = 1,
#endif
#define ADC_CONTEXT_DEFAULT { \
    .adc_raw = 0, \
    .adc_voltage = 0, \
    .do_calibration = 0, \
    .adc1_cali_handle = NULL, \
    .adc1_handle = NULL, \
    CTX_PART \
}
static adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;

static const uint16_t v_graph_lipo[V_GRAPH_LIPO_LEN] = {
    32700,  // 0
    36100,  // 5
    36900,  // 10
    37100,  // 15
    37300,  // 20
    37500,  // 25
    37700,  // 30
    37900,  // 35
    38000,  // 40
    38200,  // 45
    38400,  // 50
    38500,  // 55
    38700,  // 60
    39100,  // 65
    39500,  // 70
    39800,  // 75
    40200,  // 80
    40800,  // 85
    41100,  // 90
    41500,  // 95
    42000,  // 100
};

uint8_t calc_bat_perc_v(float adc) {
    ILOG(TAG, "[%s] %.04f", __func__, adc);
    uint32_t kadc = adc * 10000, sv, step, v, v1;
    uint8_t i=0, ret = 0, perc=0;
    if(kadc<=v_graph_lipo[0]) {
        ret = 0;
    }
    else if(kadc<=v_graph_lipo[V_GRAPH_LIPO_LEN-1]){
        for(;i<V_GRAPH_LIPO_LEN;++i, perc+=5) { // 0-100%
            v=v_graph_lipo[i]; // 32700
            v1=v_graph_lipo[i+1]; // 36100
            if(kadc == v) { // 32700
                ret = perc;
                goto done;
            }
            else if(kadc == v1) { // 36100
                ret = perc+5;
                goto done;
            }
            else if(kadc<v1) { // between 32700 and 36100
                step = (v1 - v) / 5; // divide by 1% for steps
                ++perc;
                for(sv=v+step;sv<=v1;sv+=step,++perc) {
                    if(kadc<=sv) {
                        ret = perc;
                        goto done;
                    }
                }
            }
        }
    } else ret = 100;
    done:
#if (C_LOG_LEVEL < 1)
    TLOG(TAG,"[%s] voltage: %f converted: %lu mV perc: %hhu\n", __func__, adc, kadc, ret);
#endif
    return ret;
}

// uint32_t calc_bat_perc(float adc) {
//     ILOG(TAG, "[%s] %0.4f", __func__, adc);
//     uint32_t adck = adc * 1000;
//     uint32_t bat_perc = VOLTAGE_PERC(adck);
// #if (C_LOG_LEVEL < 1)
//     DLOG(TAG, "[%s] voltage: %f converted: %lu mV perc: %lu coef: %lu\n", __func__, adc, adck, bat_perc, VOLTAGE_PERC_COEF(adck));
// #endif
//     if (bat_perc < 0)
//         bat_perc = 0;
//     else if (bat_perc > 100)
//         bat_perc = 100;
//     return bat_perc;
// }
static const char * cali_mode = "";
static uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = ESP_FAIL;
    uint8_t calibrated = false;
    adc_cali_handle_t handle = NULL;
    if (!calibrated) {
        
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
        cali_mode = "Curve Fitting";
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
        cali_mode = "Line Fitting";
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
#endif
        DLOG(TAG,"[%s] calibration scheme version is %s\n", __func__, cali_mode);
        if (ret == ESP_OK) calibrated = true;
    }
    *out_handle = handle;
#if (C_LOG_LEVEL < 1)
    if (ret == ESP_OK) {
        DLOG(TAG,"[%s] Calibration Success\n", __func__);
    } else 
#endif
    if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        WLOG(TAG, "[%s] eFuse not burnt, skip software calibration", __func__);
    } else {
        ELOG(TAG, "[%s] Invalid arg or no memory", __func__);
    }
    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
    ILOG(TAG, "[%s]", __func__);
    DLOG(TAG, "[%s] deregister %s calibration scheme\n", __func__, cali_mode);
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static uint32_t adc_read_raw() {
    esp_err_t err = 0;
    int v = 0;
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    ESP_ERROR_CHECK(adc_oneshot_read(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &v));
    adc_ctx.adc_raw = v;
#endif
    if (adc_ctx.do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_ctx.adc1_cali_handle, adc_ctx.adc_raw, &v));
        adc_ctx.adc_voltage = v;
    }
    else adc_ctx.adc_voltage = adc_ctx.adc_raw;
    TLOG(TAG, "[%s] ADC%d channel[%d]: raw: %lu, calibrated: %lu\n", __func__, _ADC_UNIT_0 + 1, _ADC_CHANNEL_0, adc_ctx.adc_raw, adc_ctx.adc_voltage);
    return adc_ctx.adc_voltage;
}

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

static uint32_t adc_read_count(uint16_t count, uint16_t delay) {
    uint32_t reading = adc_read_raw(), cur = reading;  // 2076
    for (uint16_t i = 0; i < count; i++) {
        cur = adc_read_raw(); // * 0.8 + reading * 0.2;
        reading = (cur + reading * (count - 1)) / count;
        //reading += adc_read_raw();
        if (delay) vTaskDelay((delay + (portTICK_PERIOD_MS - 1)) / portTICK_PERIOD_MS);
    }
    //return (count ? reading / count : reading) * 100;
    return reading*100;
}

static void adc_update(void*arg) {
    ILOG(TAG, "[%s]", __func__);
    uint32_t reading = VOLTAGE_CONV(adc_read_count(5, 0));
    uint32_t corr = reading;
    if(xSemaphoreTake(adc_ctx.xMutex, portMAX_DELAY)) {
        adc_ctx.result[++adc_ctx.result_index % RESULT_SIZE] = corr;
        xSemaphoreGive(adc_ctx.xMutex);
    }
#if (C_LOG_LEVEL < 1)
    for(int i=0; i<RESULT_SIZE; ++i) TLOG(TAG,"* [%s] voltage[%d]: %lu\n", __func__, i, adc_ctx.result[i]);
    TLOG(TAG,"[%s] reading: %lu corr: %lu puttoindex: %ld\n", __func__, reading, corr, (adc_ctx.result_index % RESULT_SIZE));
#endif

    esp_event_post(ADC_EVENT, ADC_EVENT_UPDATE, &corr, sizeof(corr), portMAX_DELAY);
}

#endif

#if defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_ctx.adc_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void adc_task(void * arg) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret;
    uint8_t count = 0;
    while (adc_ctx.task_is_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ret = adc_continuous_read(adc_ctx.adc1_handle, adc_ctx.result, READ_LEN, &adc_ctx.ret_num, 0);
        if (ret == ESP_OK) {
            for (int i = 0; i < adc_ctx.ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_ctx.result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);
                if (chan_num == SOC_ADC_CHANNEL_NUM(_ADC_CHANNEL_0)) adc_ctx.adc_raw = data;
            }
        }
        if (count > 10) count = 0;
        if (count == 0) {
            DLOG(TAG,"[%s] adc_raw: %lu\n", __func__, adc_ctx.adc_raw);
        }
        ++count;
        delay_ms(10);
    }
    vTaskDelete(NULL);
}

#endif

esp_err_t adc_init(void) {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t ret = 0;
    adc_ctx.do_calibration = adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN, &adc_ctx.adc1_cali_handle);
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = _ADC_UNIT_0,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_ctx.adc1_handle));
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = _ADC_BITWIDTH,
        .atten = _ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &adc_config));
    if(adc_ctx.xMutex == NULL) adc_ctx.xMutex = xSemaphoreCreateMutex();
    adc_update(0);
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &adc_update,
        .name = "periodic_adc",
        .arg = NULL
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &adc_ctx.adc_periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(adc_ctx.adc_periodic_timer, 1000000));
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    memset(&adc_ctx.result[0], 0xcc, READ_LEN);
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 256,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_ctx.adc1_handle));
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
        .pattern_num = 1,
    };

    adc_digi_pattern_config_t adc_patterns[1] = {0};
    dig_cfg.pattern_num = 0;
    adc_patterns[0].atten = _ADC_ATTEN;
    adc_patterns[0].channel = _ADC_CHANNEL_0;
    adc_patterns[0].unit = _ADC_UNIT_0;
    adc_patterns[0].bit_width = _ADC_BITWIDTH;
    DLOG(TAG, "adc_patterns[0].atten is 0x%"PRIx8"\n", adc_patterns[0].atten);
    DLOG(TAG, "adc_patterns[0].channel is 0x%"PRIx8"\n", adc_patterns[0].channel);
    DLOG(TAG, "adc_patterns[0].unit is 0x%"PRIx8"\n", adc_patterns[0].unit);
    dig_cfg.adc_pattern = adc_patterns;
    ESP_ERROR_CHECK(adc_continuous_config(adc_ctx.adc1_handle, &dig_cfg));
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    xTaskCreatePinnedToCore(adc_task, "ADC Task", (8*256), NULL, 0, &adc_ctx.adc_task_handle, 0);
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_ctx.adc1_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_ctx.adc1_handle));
    delay_ms(200);
#endif
    return ret;
}

esp_err_t adc_deinit() {
    ILOG(TAG, "[%s]", __func__);
    esp_err_t err = 0;
    if (adc_ctx.do_calibration) {
        adc_calibration_deinit(adc_ctx.adc1_cali_handle);
    }
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    ESP_ERROR_CHECK(esp_timer_stop(adc_ctx.adc_periodic_timer));
    ESP_ERROR_CHECK(esp_timer_delete(adc_ctx.adc_periodic_timer));
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_ctx.adc1_handle));
    if(adc_ctx.xMutex != NULL){
        vSemaphoreDelete(adc_ctx.xMutex);
        adc_ctx.xMutex = NULL;
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_ctx.task_is_running = 0;
    xTaskNotifyGive(adc_ctx.adc_task_handle);
    ESP_ERROR_CHECK(adc_continuous_stop(adc_ctx.adc1_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_ctx.adc1_handle));
#endif
    return err;
}

float volt_read(void) {
    ILOG(TAG, "[%s]", __func__);
    float voltage = 0;
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    voltage = VOLTAGE_U32_TO_V(((float)adc_ctx.result[adc_ctx.result_index % RESULT_SIZE])); 
    // return (float)smooth_int(&(adc_ctx.result[0]), adc_ctx.result_index, RESULT_SIZE, 1) / 1000/100;
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    voltage = VOLTAGE_U32_TO_V((VOLTAGE_CONV((float)adc_ctx.adc_raw)));
#endif
    DLOG(TAG, "[%s] adc_raw: %lu volt: %f\n", __func__, adc_ctx.adc_raw, voltage);
    return voltage;
}

#endif // CONFIG_LOGGER_ADC_ENABLED