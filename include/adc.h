#ifndef F591E13F_5F2C_4BDB_9203_BB5C407DF403
#define F591E13F_5F2C_4BDB_9203_BB5C407DF403

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#if defined(CONFIG_LOGGER_ADC_ENABLED)

int init_adc();
int deinit_adc();
int adc_read();
#ifdef USE_CUSTOM_CALIBRATION_VAL
float volt_read(float calibration);
#else
float volt_read();
#endif
int32_t adc_read_count(uint16_t count, uint16_t delay);
int calc_bat_perc(float adc);
uint8_t calc_bat_perc_v(float adc);

#else

#define init_adc() (void)0
#define deinit_adc() (void)0
#define adc_read() 0
#define volt_read() 0.0f
#define adc_read_count(a, b) 0
#define calc_bat_perc(a) 0
#define calc_bat_perc_v(a) 0u

#endif

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
