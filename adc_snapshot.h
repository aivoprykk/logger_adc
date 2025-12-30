#ifndef ACC17C56_AE1B_4DBF_ABB4_766EE45A4140
#define ACC17C56_AE1B_4DBF_ABB4_766EE45A4140

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adc_private_defs.h"
#include "adc_battery_adp.h"

typedef struct {
    uint16_t voltage_3V2;  /* Critical low threshold (raw ADC) */
    uint16_t voltage_3V6;   /* Discharging range start */
    uint16_t voltage_3V8;   /* Nominal range start */
    uint16_t voltage_4V0;   /* Charging range start */
    uint16_t voltage_4V1;   /* Full range start */
    uint16_t voltage_4V2;   /* Maximum voltage */
} battery_calibration_t;

extern battery_calibration_t current_calibration;

bool is_calibration_applied(void);
void apply_battery_calibration(uint16_t raw_3V25, uint16_t raw_4V2);
void battery_update_calibration(battery_calibration_t* calib, uint16_t raw_min, uint16_t raw_max);
void battery_set_calibration(const battery_calibration_t* calibration);
const battery_calibration_t * battery_get_calibration(void);
void battery_set_default_calibration(void);
void battery_dump_calibration(const battery_calibration_t* calibration, const char* context);
void c_dump_calibration(void);

typedef struct battery_snapshot_s {
    uint32_t snapshot_timestamp;
    battery_monitor_t battery_monitor;
    // Snapshot Source identification
    // monitor_source_t data_source;  // 0=ULP, 1=C, 2=Hybrid
    uint8_t snapshot_valid;        // 1=valid data, 0=stale
    // battery_calibration_t * calibration;
} battery_snapshot_t;

#define BATTERY_SNAPSHOT_DEFAULTS() { 0, \
    BATTERY_MONITOR_DEFAULTS(&ADC_STATE_DEFAULTS, &ADC_STATE_DEFAULTS, &ADC_RUNNING_AVG_DEFAULTS, &PLATEAU_DEFAULTS), 0 \
}
void monitor_dump(const battery_monitor_t *monitor, const char* tag);
void snapshot_dump(const battery_snapshot_t *snapshot, const char* tag);

// Unified API
void battery_snapshot_init(void);
battery_snapshot_t* battery_get_snapshot(void);

uint8_t should_wake_for_battery_state(uint8_t new_battery_state, uint8_t last_packed_status, uint8_t main_cpu_running);
uint8_t battery_set_battery_state(battery_monitor_t* monitor, adc_battery_state_t new_state, bool update_last);

// ULP-specific interface
#ifdef ULP_MODE
    void update_ulp_calibration(const battery_calibration_t* calibration);
#else
    battery_monitor_t* battery_get_monitor_handle(void);
    void battery_handle_button_event(adc_button_state_t button_state);
#endif

#ifdef __cplusplus
}
#endif


#endif /* ACC17C56_AE1B_4DBF_ABB4_766EE45A4140 */
