#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_ENABLED) && defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

#include "esp_adc/adc_oneshot.h"
#include "ulp_config.h"

static const char *TAG = "adc_oneshot";
static bool adc_oneshot_initialized = false;
adc_oneshot_unit_handle_t adc1_handle = NULL;
static const char * cali_mode = "";
static uint8_t read_count = 4;
static uint16_t read_delay_ms = 2; // ms

uint32_t read_battery_adc(void) {
    uint32_t sum = 0;
    uint32_t min_val = UINT32_MAX;
    uint32_t max_val = 0;
    int v = 0, c = read_count;
    // Single pass: calculate sum and find min/max for outlier rejection
    for (uint16_t i = 0; i < read_count; i++) {
        if(!adc1_handle || adc_oneshot_read(adc1_handle, _ADC_CHANNEL_0, &v)) {
            ELOG(TAG, "[%s] Failed to read ADC %d", __func__, _ADC_CHANNEL_0);
            return 0;
        }
        sum += (uint32_t)v;

        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;

        if (read_delay_ms) vTaskDelay(pdMS_TO_TICKS(read_delay_ms));
    }

    // Simple outlier rejection: remove min and max if we have enough samples
    if (c >= 4) {
        sum = sum - min_val - max_val;
        c -= 2;
    }
    uint32_t ret = (sum / c);
    DLOG(TAG, "[%s] %lu (avg of %u samples)", __func__, ret, c);
    return ret;
}

esp_err_t adc_oneshot_init(void) {
    FUNC_ENTRY(TAG);
    if(adc_oneshot_initialized) return ESP_OK; // Already initialized
    esp_err_t ret = 0;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = _ADC_UNIT_0,
    };
    if(adc_oneshot_new_unit(&init_config1, &adc1_handle)) {
        ELOG(TAG, "[%s] Failed to create ADC unit", __func__);
        return ESP_FAIL;
    }
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = _ADC_BITWIDTH,
        .atten = _ADC_ATTEN,
    };
    if(adc_oneshot_config_channel(adc1_handle, _ADC_CHANNEL_0, &adc_config)) {
        ELOG(TAG, "[%s] Failed to config ADC channel", __func__);
        return ESP_FAIL;
    }
    adc_oneshot_initialized = true;
    return ret;
}

esp_err_t adc_oneshot_deinit(void) {
    FUNC_ENTRY(TAG);
    if(!adc_oneshot_initialized) return ESP_OK; // Not initialized
    adc_oneshot_initialized = false;
    esp_err_t err = 0;
    if(adc_lock(-1)) {
        adc_unlock();
    }

    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    return err;
}

#endif // CONFIG_LOGGER_ADC_ENABLED
