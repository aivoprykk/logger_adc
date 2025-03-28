#ifndef F591E13F_5F2C_4BDB_9203_BB5C407DF403
#define F591E13F_5F2C_4BDB_9203_BB5C407DF403

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
int adc_init();
int adc_deinit();
float volt_read();
uint8_t calc_bat_perc_v(float adc);

#ifdef __cplusplus
}
#endif
#endif /* F591E13F_5F2C_4BDB_9203_BB5C407DF403 */
