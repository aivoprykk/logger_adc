#ifndef C55CD9D1_A8E5_40D7_B1C8_3A17B0AAB749
#define C55CD9D1_A8E5_40D7_B1C8_3A17B0AAB749

#ifdef __cplusplus
extern "C" {
#endif

#include "adc_snapshot.h"

void compute_and_store_ulp_thresholds(void);
void ulp_live_snap_init(void);
battery_snapshot_t * ulp_live_snap_take(void);
battery_snapshot_t * ulp_live_snap_take_wait(uint32_t timeout_ms);
battery_snapshot_t * ulp_live_snap_get(void);
const battery_monitor_t * ulp_live_snap_get_monitor(void);
void ulp_take_last_snapshot(battery_snapshot_t *snap, uint32_t timeout_ms);
void ulp_dump_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* C55CD9D1_A8E5_40D7_B1C8_3A17B0AAB749 */
