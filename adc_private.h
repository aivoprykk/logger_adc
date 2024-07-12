#ifndef C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A
#define C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_adc/adc_cali.h"

void volt_update();
void E_adc_calibration_deinit(adc_cali_handle_t handle);
unsigned char E_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
#ifdef USE_CUSTOM_CALIBRATION_VAL
void volt_update(float calibration);
#else
void volt_update();
float convertVoltage(int32_t volt);
#endif

#if (CONFIG_LOGGER_ADC_LOG_LEVEL <= 2)

#include "esp_timer.h"
#include "esp_log.h"

#ifndef LOG_INFO
#define LOG_INFO(a, b, ...) ESP_LOGI(a, b, __VA_ARGS__)
#endif
#ifndef MEAS_START
#define MEAS_START() uint64_t _start = (esp_timer_get_time())
#endif
#ifndef MEAS_END
#define MEAS_END(a, b, ...) \
    ESP_LOGI(a, b, __VA_ARGS__, (esp_timer_get_time() - _start))
#endif
#endif

#if defined(CONFIG_LOGGER_ADC_LOG_LEVEL_TRACE) // "A lot of logs to give detailed information"

#define DLOG LOG_INFO
#define DMEAS_START MEAS_START
#define DMEAS_END MEAS_END
#define ILOG LOG_INFO
#define IMEAS_START MEAS_START
#define IMEAS_END MEAS_END
#define WLOG LOG_INFO
#define WMEAS_START MEAS_START
#define WMEAS_END MEAS_END

#elif defined(CONFIG_LOGGER_ADC_LOG_LEVEL_INFO) // "Log important events"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG LOG_INFO
#define IMEAS_START MEAS_START
#define WLOG LOG_INFO
#define WMEAS_START MEAS_START
#define WMEAS_END MEAS_END

#elif defined(CONFIG_LOGGER_ADC_LOG_LEVEL_WARN) // "Log if something unwanted happened but didn't cause a problem"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG(a, b, ...) ((void)0)
#define IMEAS_START() ((void)0)
#define IMEAS_END(a, b, ...) ((void)0)
#define WLOG LOG_INFO
#define WMEAS_START MEAS_START
#define WMEAS_END MEAS_END

#else // "Do not log anything"

#define DLOG(a, b, ...) ((void)0)
#define DMEAS_START() ((void)0)
#define DMEAS_END(a, b, ...) ((void)0)
#define ILOG(a, b, ...) ((void)0)
#define IMEAS_START() ((void)0)
#define IMEAS_END(a, b, ...) ((void)0)
#define WLOG(a, b, ...) ((void)0)
#define WMEAS_START() ((void)0)
#define WMEAS_END(a, b, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */
