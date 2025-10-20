#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include "sdkconfig.h"

#if CONFIG_LOGGER_ADC_ENABLED

#include "adc.h"
#include "adc_events.h"
#include "math.h"

#define MINIMUM_VOLTAGE 3.25
#define BAT_LOW_TRESHOLD 3.4
#define BAT_UP_TRESHOLD 4.0

static const char *TAG = "demo_adc";
RTC_DATA_ATTR float rtc_voltage_bat = 0.00f;

static bool update_rtc_bat_from_ulp(void) {
#if defined(CONFIG_ULP_COPROC_ENABLED)
    uint32_t ulp_mv = get_ulp_adc_calibrated();
    if (ulp_mv > 0 && ulp_mv < 4096) {
        float old_voltage = rtc_voltage_bat;
        rtc_voltage_bat = (float)ulp_mv / 1000.0f;
        ESP_LOGD(TAG, "[%s] Using ULP reading: raw=%lu, %lu mV (%.02fV)", __FUNCTION__, get_ulp_adc_reading(), ulp_mv, rtc_voltage_bat);
        // Log significant changes
        if (fabs(rtc_voltage_bat - old_voltage) > 0.1f) {
            ESP_LOGD(TAG, "[%s] Battery voltage changed: %.02fV -> %.02fV", __FUNCTION__, old_voltage, rtc_voltage_bat);
        }
        return true;
    }
#endif
    return false;
}

static void update_bat(void) {
    // Use ULP reading if available (more efficient and survives deep sleep)
    if (!update_rtc_bat_from_ulp()) {
        // Fallback to direct ADC reading if ULP not available
        rtc_voltage_bat = adc_get_cached_batt_volt();
#if defined(CONFIG_ULP_COPROC_ENABLED)
        ESP_LOGD(TAG, "[%s] Using direct ADC reading: %.02fV", __FUNCTION__, rtc_voltage_bat);
#endif
    }

    ESP_LOGI(TAG, "[%s] computed:%.02f, required_min:%.02f\n", __FUNCTION__, rtc_voltage_bat, MINIMUM_VOLTAGE);
    // if(rtc_voltage_bat < MINIMUM_VOLTAGE) {
    //     ESP_LOGW(TAG, "[%s] low battery detected, start shutdown sequence: %.02f", __FUNCTION__, rtc_voltage_bat);
    //     low_bat_start_sequence();
    // }
    // else if(m_app_ctx.low_bat_countdown) {
    //     ESP_LOGW(TAG, "[%s] battery level restored, cancel shutdown sequence: %.02f", __FUNCTION__, rtc_voltage_bat);
    //     m_app_ctx.low_bat_countdown = 0;
    // }
}
#else
#include "myadc.h"
#endif

void app_main(void)
{
#if CONFIG_LOGGER_ADC_ENABLED
    adc_init();
#endif

    while (1) {

#if CONFIG_LOGGER_ADC_ENABLED
        // update_bat();
        vTaskDelay(pdMS_TO_TICKS(1000));
#else
    /* If user is using USB-serial-jtag then idf monitor needs some time to
    *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
    *  before we print anything. Otherwise the chip will go back to sleep again before the user
    *  has time to monitor any output.
    */
    vTaskDelay(pdMS_TO_TICKS(1000));

    take_measurement();

#if !CONFIG_IDF_TARGET_ESP32
    /* RTC peripheral power domain needs to be kept on to keep SAR ADC related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#endif
    esp_deep_sleep_start();
#endif

    }
}
