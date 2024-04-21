#ifndef F591E13F_5F2C_4BDB_9203_BB5C407DF403
#define F591E13F_5F2C_4BDB_9203_BB5C407DF403

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
