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

#ifdef __cplusplus
}
#endif

#endif /* C68D7F37_A55C_4F0E_A19C_B0D2B1853F2A */
