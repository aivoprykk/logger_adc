#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"
#include <esp_idf_version.h>
#include <esp_log.h>

#include "adc.h"
#include "adc_private.h"
#include "adc_events.h"
#include "logger_common.h"

#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

ESP_EVENT_DEFINE_BASE(ADC_EVENT);

const static char *TAG = "adc";

#define VOLTAGE_MAX 4190
#define VOLTAGE_MIN 3300

#define DEFAULT_VREF 1114
#define NO_OF_SAMPLES 64

#if ESP_IDF_VERSION_MAJOR < 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR <= 1 && ESP_IDF_VERSION_PATCH < 3)
#define ADC_ATTEN ADC_ATTEN_DB_11
#else
#define ADC_ATTEN ADC_ATTEN_DB_12
#endif

#if CONFIG_IDF_TARGET_ESP32
#define E_ADC1_CHAN0 ADC_CHANNEL_7
#if E_USE_ADC1_2
#define E_ADC1_CHAN1 ADC_CHANNEL_5
#endif
#else
#define E_ADC1_CHAN0 ADC_CHANNEL_3
#if E_USE_ADC1_2
#define E_ADC1_CHAN1 ADC_CHANNEL_0
#endif
#endif

#define VOLTAGE_ROW_SIZE 6
#define V_GRAPH_LIPO_LEN 21
#define ADJ_LENGTH 24

typedef struct adc_context_s {
    esp_timer_handle_t adc_periodic_timer;
    int adc_raw;
    int voltage;
    int voltage_row[VOLTAGE_ROW_SIZE];
    int32_t voltage_row_index;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle;
    unsigned char do_calibration1_chan0;
    SemaphoreHandle_t xMutex;
} adc_context_t;

#define ADC_CONTEXT_DEFAULT { \
    .adc_periodic_timer = NULL, \
    .adc_raw = 0, \
    .voltage = 0, \
    .voltage_row = {0}, \
    .voltage_row_index = -1, \
    .adc1_handle = NULL, \
    .adc1_cali_chan0_handle = NULL, \
    .do_calibration1_chan0 = 0, \
    .xMutex = NULL \
}

static adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;

uint16_t v_graph_lipo[V_GRAPH_LIPO_LEN] = {
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

#define USE_CORRECTION 1
#ifdef USE_CORRECTION

uint16_t td_s3_adj[ADJ_LENGTH] = {
#if defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3)
//lilygo t-display s3 calibration data
    2958,//3100
    3002,//3150
    3055,//3200
    3107,//3250
    3166,//3300
    3224,//3350
    3285,//3400
    3339,//3450
    3393,//3500
    3440,//3550
    3498,//3600
    3546,//3650
    3600,//3700
    3645,//3750
    3700,//3800
    3748,//3850
    3800,//3900
    3848,//3950
    3900,//4000
    3945,//4050
    3998,//4100
    4050,//4150
    4094,//4200
    4148,//4250
#elif defined(CONFIG_HAS_BOARD_LILYGO_EPAPER_T5)
/// lilygo t5 v213 adc calibration data
    //2987,//3050
    3040,//3100
    3095,//3150
    3147,//3200
    3201,//3250
    3255,//3300
    3303,//3350
    3355,//3400
    3405,//3450
    3455,//3500
    3503,//3550
    3554,//3600
    3602,//3650
    3652,//3700
    3709,//3750
    3757,//3800
    3803,//3850
    3851,//3900
    3902,//3950
    3952,//4000
    4001,//4050
    4052,//4100
    4100,//4150
    4150,//4200
    4198,//4250
#endif
};
#endif

uint8_t calc_bat_perc_v(float adc) {
    uint16_t kadc = adc * 10000, sv, step;
    uint8_t i=0, ret = 0, perc=0;
    uint16_t v, v1;
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
    } else {
        ret = 100;
    }
    done:
    ESP_LOGI(TAG, "[%s] (voltage: %f, converted: %"PRIu16" mV, perc: %"PRIu8")", __FUNCTION__, adc, kadc, ret);
    return ret;
}

int calc_bat_perc(float adc) {
    float adck = adc * 1000;
    float bat_perc = (100 * ((1 - ((VOLTAGE_MAX - adck) / (VOLTAGE_MAX - VOLTAGE_MIN)))));
    ESP_LOGI(TAG, "[%s] (voltage: %f, converted: %f mV, perc: %f, coef: %f)", __FUNCTION__, adc, adck, bat_perc, 1-((VOLTAGE_MAX - adck) / (VOLTAGE_MAX - VOLTAGE_MIN)));

    if (bat_perc < 0)
        bat_perc = 0;
    else if (bat_perc > 100)
        bat_perc = 100;
    return (int)bat_perc;
}

unsigned char E_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    unsigned char calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
    return calibrated;
}

void E_adc_calibration_deinit(adc_cali_handle_t handle) {
    esp_err_t err = 0;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    err = adc_cali_delete_scheme_curve_fitting(handle);
    if (err) {
        ESP_LOGE(TAG, "failed to deregister %s calibration scheme",
                 "Line Fitting");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    err = adc_cali_delete_scheme_line_fitting(handle);
    if (err) {
        ESP_LOGE(TAG, "failed to deregister %s calibration scheme",
                 "Line Fitting");
    }
#endif
}

static void periodic_timer_callback(void* arg) {
    volt_update();
}

int init_adc() {
    esp_err_t err = 0;
    int ret = 0;
    LOGR;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    err = adc_oneshot_new_unit(&init_config1, &adc_ctx.adc1_handle);
    if (err) {
        ESP_LOGE(TAG, "failed to adc_oneshot_new_unit adc1");
    }

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    err = adc_oneshot_config_channel(adc_ctx.adc1_handle, E_ADC1_CHAN0, &config);
    if (err) {
        ESP_LOGE(TAG, "failed to adc_oneshot_config_channel adc1 chan0");
    }

    //-------------ADC1 Calibration Init---------------//
    adc_ctx.do_calibration1_chan0 = E_adc_calibration_init(ADC_UNIT_1, E_ADC1_CHAN0, ADC_ATTEN, &adc_ctx.adc1_cali_chan0_handle);

    if(adc_ctx.xMutex == NULL)
        adc_ctx.xMutex = xSemaphoreCreateMutex();
    volt_update();
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic_adc",
        .arg = NULL
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &adc_ctx.adc_periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(adc_ctx.adc_periodic_timer, 1000000));
    return ret;
}

int deinit_adc() {
    esp_err_t err = 0;
    ESP_ERROR_CHECK(esp_timer_delete(adc_ctx.adc_periodic_timer));
    // Tear Down
    err = adc_oneshot_del_unit(adc_ctx.adc1_handle);
    if (err) {
        ESP_LOGE(TAG, "failed to adc_oneshot_del_unit adc1");
    }
    if (adc_ctx.do_calibration1_chan0) {
        E_adc_calibration_deinit(adc_ctx.adc1_cali_chan0_handle);
    }
    if(adc_ctx.xMutex != NULL)
        vSemaphoreDelete(adc_ctx.xMutex);
    return err;
}

// voltage divider with 100K resistors for lilygo bat sense
#define HIGH_RESISTOR 100000L
#define LOW_RESISTOR 100000L

float convertVoltage(int32_t volt) {
    return (HIGH_RESISTOR + LOW_RESISTOR) / LOW_RESISTOR * (volt / 100);
}

int32_t adc_read_count(uint16_t count, uint16_t delay) {
    int reading = adc_read(), cur = reading;  // 2076
    for (uint16_t i = 0; i < count; i++) {
        cur = adc_read() * 0.8 + reading * 0.2;
        reading = (cur + reading * (count - 1)) / count;
        //reading += adc_read();
        if (delay)
            delay_ms(delay);
    }
    //return (count ? reading / count : reading) * 100;
    return reading*100;
}

#ifdef USE_CUSTOM_CALIBRATION_VAL
void volt_update(float calibration) {
#else
void volt_update() {
#endif
    float reading = convertVoltage(adc_read_count(5, 0));
#ifdef USE_CUSTOM_CALIBRATION_VAL
    float corr = (calibration == 0||calibration==1) ? reading : reading * calibration;
#else
    float corr = reading;
#endif
    float adj = 1;
#ifdef USE_CORRECTION
    uint8_t i = 0, j=ADJ_LENGTH;
    while(i<j) {
        if(reading<=td_s3_adj[i]) {
            break;
        }
        ++i;
    }
    if(i==j) --i;
    adj = (float)((3100 + (i*50)) / td_s3_adj[i]);
    //adj = td_s3_adj[i].adjustment;
#endif
    if(adj!=1&&adj!=0)
        corr *= adj;
    if(xSemaphoreTake(adc_ctx.xMutex, portMAX_DELAY)) {
        adc_ctx.voltage_row[++adc_ctx.voltage_row_index % VOLTAGE_ROW_SIZE] = (uint32_t)corr*100;
        xSemaphoreGive(adc_ctx.xMutex);
    }
// #ifdef USE_CUSTOM_CALIBRATION_VAL
//     ESP_LOGD(TAG, "[%s] Voltage computed: %f, calibration: %f,  corr: %f, adj: %f", __FUNCTION__, reading, calibration, corr, adj);
// #else
     ESP_LOGI(TAG, "[%s] Voltage computed: %f,  corr: %f, adj: %f", __FUNCTION__, reading, corr, adj);
// #endif
    esp_event_post(ADC_EVENT, ADC_EVENT_VOLTAGE_UPDATE, &corr, sizeof(corr), portMAX_DELAY);
}

#ifdef USE_CUSTOM_CALIBRATION_VAL
float volt_read(float calibration) {
#else
float volt_read() {
#endif
    if(adc_ctx.voltage_row_index < 3)
        return (float)adc_ctx.voltage_row[adc_ctx.voltage_row_index % VOLTAGE_ROW_SIZE]/1000/100;
    return (float)smooth_int(&(adc_ctx.voltage_row[0]), adc_ctx.voltage_row_index, VOLTAGE_ROW_SIZE, 1) / 1000/100;
}

int adc_read() {
    esp_err_t err = 0;
    // ESP_LOGD(TAG,"[%s]", __FUNCTION__);
    uint16_t adc_reading = 0;
    err = adc_oneshot_read(adc_ctx.adc1_handle, E_ADC1_CHAN0, &adc_ctx.adc_raw);
    if (err) {
        ESP_LOGE(TAG, "failed to adc_oneshot_read adc1 chan0");
    }
    ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, E_ADC1_CHAN0, adc_ctx.adc_raw);
    if (adc_ctx.do_calibration1_chan0) {
        err = adc_cali_raw_to_voltage(adc_ctx.adc1_cali_chan0_handle, adc_ctx.adc_raw, &adc_ctx.voltage);
        if (err) {
            ESP_LOGE(TAG, "failed to adc_cali_raw_to_voltage adc1 chan0");
        }
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, E_ADC1_CHAN0, adc_ctx.voltage);
    }

    adc_reading = adc_ctx.voltage;
    return adc_reading;
}