#ifndef D632DD21_245B_4906_8A0A_7441AA4CA69C
#define D632DD21_245B_4906_8A0A_7441AA4CA69C

#if __cplusplus
extern "C" {
#endif
#include "sdkconfig.h"
#include "stddef.h"
#include "stdint.h"

typedef struct {
    uint16_t * samples;
    uint16_t size;
    uint16_t shift;
    uint16_t count;
    uint16_t idx;
    uint16_t avg;
    uint32_t sum;
} __attribute__((__packed__)) adc_running_avg_t;

#define ADC_RUNNING_AVG_DEFAULTS (adc_running_avg_t){ \
    (uint16_t[ULP_ADC_HISTORY_SIZE]){0}, ULP_ADC_HISTORY_SIZE, ULP_ADC_HISTORY_SHIFT, 0, 0, 0, 0 \
}

#if !defined(CONFIG_ULP_COPROC_ENABLED)
void running_avg_init(adc_running_avg_t * avg, uint16_t * buffer, uint16_t size);
void running_avg_update(adc_running_avg_t * avg, uint16_t new_sample);
#endif
uint32_t running_avg_mad(const adc_running_avg_t * avg);
void running_avg_print(const adc_running_avg_t * avg, const char* name);

typedef struct {
    uint16_t last_sample; // Last sample processed for delta calculations
    int16_t  direction;
    uint16_t count;     // Capped at PLATEAU_COUNT_MAX for memory efficiency
    int16_t delta; // Last delta value
    int16_t delta_sum; // sum of deltas during plateau
    int16_t delta_avg; // average delta during plateau
    uint16_t adaptive_threshold; // adaptive threshold based on plateau stability
    uint8_t reported;  // Flag to only report plateau once
    uint8_t processing;
} __attribute__((__packed__)) adc_plateau_t;

#define PLATEAU_DEFAULTS (adc_plateau_t){0,0,0,0,0,0,0}

void plateau_print(const adc_plateau_t* plateau, const char* name);

typedef struct {
    // Current state
    uint8_t curr; // current  state: source bits + status bits
    uint8_t last;    // previous state: source bits + status bits
    uint8_t event_pending;
    uint8_t pad2;
} __attribute__((__packed__)) adc_current_state_t;

#define ADC_STATE_DEFAULTS (adc_current_state_t){0, 0, 0, 0}

void adc_current_state_print(const adc_current_state_t* state, const char* name);
inline uint8_t get_adc_state_curr(adc_current_state_t * bs) {return bs ? bs->curr : 0;}
inline uint8_t get_adc_state_last(adc_current_state_t * bs) {return bs ? bs->last : 0;}
inline uint8_t get_adc_state_event_pending(adc_current_state_t * bs) {return bs ? bs->event_pending : 0;}

typedef enum {
    MON_SR_ULP = 0,
    MON_SR_C = 1,
    MON_SR_HYBRID = 2
} monitor_source_t;

typedef struct {
    // uint16_t charging_active;
    uint16_t voltage_raw;
    monitor_source_t monitor_src;
    adc_current_state_t * battery_state;
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    adc_current_state_t * button_state;
#endif
    adc_running_avg_t * slow_window;
    adc_plateau_t * plateau;
} __attribute__((__packed__)) battery_monitor_t;

#if defined(CONFIG_ULP_BUTTON_ENABLED)
#define BATTERY_MONITOR_DEFAULTS(monitor_state_ptr, button_state_ptr, slow_window_ptr, plateau_ptr) (battery_monitor_t){ \
    0, 0, monitor_state_ptr, button_state_ptr, slow_window_ptr, plateau_ptr \
}
#else
#define BATTERY_MONITOR_DEFAULTS(monitor_state_ptr, button_state_ptr, slow_window_ptr, plateau_ptr) (battery_monitor_t){ \
    0, 0, monitor_state_ptr, slow_window_ptr, plateau_ptr \
}
#endif

void battery_monitor_init(battery_monitor_t* monitor, 
    adc_plateau_t * plateau, 
    adc_running_avg_t * slow_window, 
    adc_current_state_t * battery_state
#ifdef CONFIG_ULP_BUTTON_ENABLED
    , adc_current_state_t * button_state
#endif
    );

#if defined(CONFIG_ULP_COPROC_ENABLED)
// Public API
uint8_t battery_monitor_update(battery_monitor_t* monitor, uint16_t adc_reading);

// Helper functions
uint16_t get_directional_threshold(uint16_t voltage, int16_t direction);
uint16_t get_delta_min(uint16_t voltage, int16_t direction);

void c_live_snap_init(void);
struct battery_snapshot_s * c_live_snap_update(uint16_t adc_reading);
struct battery_snapshot_s * c_live_snap_get(void);
battery_monitor_t * c_live_snap_get_monitor(void);
#endif

#if __cplusplus
}
#endif

#endif /* D632DD21_245B_4906_8A0A_7441AA4CA69C */
