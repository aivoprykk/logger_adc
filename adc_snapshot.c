#include "adc_private.h"

#include "adc_snapshot.h"
#include "adc_ulp.h"

#include "esp_timer.h"
#include "string.h"

#include "ulp_program.h"

static const char *TAG = "adc_snapshot";

static bool snapshot_initialized = 0;
RTC_DATA_ATTR static bool calibration_applied = false;

battery_calibration_t current_calibration = {0};
static battery_snapshot_t current_snapshot = {0};

void battery_snapshot_init(void) {
    if(snapshot_initialized) return;
    FUNC_ENTRYD(TAG);
#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)
    c_live_snap_init();
#endif
#ifndef ULP_MODE
    current_snapshot.data_source = 2; // Hybrid by default
    // memset(&c_snapshot, 0, sizeof(battery_snapshot_t));
    // c_snapshot.calibration = &current_calibration;
#endif
    snapshot_initialized = 1;
}

uint8_t should_wake_for_battery_state(uint8_t new_battery_state, uint8_t last_packed_status, uint8_t main_cpu_running) {
    // Always wake if main CPU is already running
    if (main_cpu_running) {
        return 1;
    }
    // Unpack last status
    uint8_t last_bat_status = battery_get_last_battery_state();
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    uint8_t last_but_status = battery_get_last_button_state();
#endif
    uint8_t last_source = last_bat_status ? WAKE_SOURCE_BATTERY :
#if defined(CONFIG_ULP_BUTTON_ENABLED) 
        last_but_status ? WAKE_SOURCE_BUTTON : 
#endif
        WAKE_SOURCE_NONE;

    // Rule 1: Always wake for charging state changes during sleep
    if (new_battery_state == ADC_BATTERY_CHARGING) {
        return 1;
    }
    
    // Rule 2: Don't wake for critical low if last was also critical low
    if (new_battery_state == ADC_BATTERY_CRITICAL_LOW) {
        if (last_source == WAKE_SOURCE_BATTERY && last_bat_status == ADC_BATTERY_CRITICAL_LOW) {
            return 0; // Suppress repeated critical low
        }
        return 1; // First critical low
    }
    
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    // Rule 3: Wake for button state changes
    if (last_source == WAKE_SOURCE_BUTTON) {
        return 1;
    }
 #endif
    return 0;
}

inline bool is_calibration_applied(void) {
    return calibration_applied;
}

const battery_calibration_t * battery_get_calibration(void) {
    return &current_calibration;
}

void battery_set_calibration(const battery_calibration_t* calibration) {
#ifdef ULP_MODE
    update_ulp_calibration(calibration);
#endif
    memcpy(&current_calibration, calibration, sizeof(battery_calibration_t));
}

void battery_dump_calibration(const battery_calibration_t* calibration, const char* context) {
    if(!calibration) return;
    ILOG(TAG, "%s Battery Calibration: 3.2V: %hu, 3.6V: %hu, 3.8V: %hu, 4.0V: %hu, 4.1V: %hu, 4.2V: %hu", 
        context, calibration->voltage_3V2, calibration->voltage_3V6, calibration->voltage_3V8, 
        calibration->voltage_4V0, calibration->voltage_4V1, calibration->voltage_4V2);
}

void battery_update_calibration(battery_calibration_t* calib, uint16_t raw_min, uint16_t raw_max) {
    // Calculate intermediate points based on calibration
    uint16_t diff = raw_max - raw_min;
    calib->voltage_3V2 = raw_min;
    calib->voltage_3V6 = raw_min + (diff * 4) / 10;
    calib->voltage_3V8 = raw_min + (diff * 6) / 10;
    calib->voltage_4V0 = raw_min + (diff * 8) / 10;
    calib->voltage_4V1 = raw_min + (diff * 9) / 10;
    calib->voltage_4V2 = raw_max;
    calibration_applied = true;
}

void apply_battery_calibration(uint16_t raw_3V25, uint16_t raw_4V2) {
    battery_calibration_t calib;
    battery_update_calibration(&calib, raw_3V25, raw_4V2);
    battery_set_calibration(&calib);
    battery_dump_calibration(&calib, "Applied");
}

void c_dump_calibration(void) {
    battery_dump_calibration(&current_calibration, "Current");
}

#ifdef ULP_MODE

void update_ulp_calibration(const battery_calibration_t* calibration) {
    // Always update ULP calibration values in RTC memory
    ULP_SET_U32(ulp_calibrated_voltage_3V2, calibration->voltage_3V2);
    ULP_SET_U32(ulp_calibrated_voltage_3V6, calibration->voltage_3V6);
    ULP_SET_U32(ulp_calibrated_voltage_3V8, calibration->voltage_3V8);
    ULP_SET_U32(ulp_calibrated_voltage_4V0, calibration->voltage_4V0);
    ULP_SET_U32(ulp_calibrated_voltage_4V1, calibration->voltage_4V1);
    ULP_SET_U32(ulp_calibrated_voltage_4V2, calibration->voltage_4V2);
    battery_dump_calibration(calibration, "ULP Calib Updated");
}

static void update_snapshot_from_ulp(battery_snapshot_t* snapshot) {
    FUNC_ENTRYD(TAG);
    uint32_t valid = ULP_GET_U32(ulp_snapshot_valid);
    uint32_t cycle_count = ULP_GET_U32(ulp_cycle_count);
    DLOG(TAG, "ULP consume snapshot from: %s", valid ? "snap" : "live");
    if (valid) {
        snapshot->battery_monitor.voltage_raw = ULP_GET_U32(ulp_snapshot_voltage);
        snapshot->snapshot_timestamp = ULP_GET_U32(ulp_snapshot_timestamp);
        ULP_SET_U32(ulp_snapshot_valid, 0);
    } else {
        snapshot->battery_monitor.voltage_raw = ULP_GET_U32(ulp_last_result);
        snapshot->snapshot_timestamp = cycle_count;
    }
    snapshot->battery_monitor.monitor_src = MON_SR_ULP;
    snapshot->snapshot_valid = 1;
}

#endif

#ifndef ULP_MODE
static void update_snapshot_from_c(battery_snapshot_t* snapshot, uint16_t adc_reading) {
    FUNC_ENTRY_ARGS(TAG, "adc_reading=%u", adc_reading);
    // Populate C snapshot from authoritative battery_monitor
    battery_monitor_t* m = &snapshot.battery_monitor;
    // monitor->charging_active = battery_monitor.charging_active;
    m->voltage_raw = adc_reading; // raw ADC reading
    // monitor->curr_battery_state = battery_monitor->battery_state->curr;
    // monitor->last_battery_state = battery_monitor.last_battery_state;
    m->monitor_src = MON_SR_C;
    snapshot.snapshot_timestamp = get_millis();
    snapshot.data_source = 1; // C source
    snapshot.snapshot_valid = 1;
}

#ifdef CONFIG_ULP_BUTTON_ENABLED
void battery_handle_button_event(battery_monitor_t* monitor , button_state_t button_state) {
    // Update state for button press
    monitor->last_battery_state = monitor->curr_battery_state;
    set_button_status(monitor->curr_battery_state, button_state);
    // monitor->state_change_count++;
    // monitor->last_battery_state_change = esp_timer_get_time();
    monitor->snapshot_timestamp = esp_timer_get_time();
    
    // Update common snapshot
    // memcpy(&current_snapshot, &c_snapshot, sizeof(battery_snapshot_t));
}
#endif
#endif

void debug_ulp_status(void) {
#ifdef ULP_MODE
    if(!ulp_prog_is_initialized()) return;
    snapshot_dump(&current_snapshot, "ULP Snap");
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
    ulp_dump_calibration();
#endif
#endif
}

void debug_ulp_get_status(void) {
#ifdef ULP_MODE
    if(!ulp_prog_is_initialized()) return;
    snapshot_dump(ulp_live_snap_get(), "ULP Snap");
#if (C_LOG_LEVEL <= LOG_DEBUG_NUM)
    ulp_dump_calibration();
#endif
#endif
}

void debug_c_status(void) {
#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)
    // snapshot_dump(c_live_snap_get(), "C Live");
#endif
#ifndef ULP_MODE
    snapshot_dump(&current_snapshot, "C Snap");
#endif
}

battery_snapshot_t* battery_get_snapshot(void) {
    FUNC_ENTRYD(TAG);
    return &current_snapshot;
}

battery_snapshot_t* battery_update_snapshot(uint16_t adc_reading) {
    FUNC_ENTRYD(TAG);
#ifdef ULP_MODE
    update_snapshot_from_ulp(&current_snapshot);
    ulp_live_snap_take_wait(CONFIG_ADC_CYCLE_TIME_MS);
#endif
#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)
    c_live_snap_update(adc_reading);
#endif
#ifndef ULP_MODE
    update_snapshot_from_c(&current_snapshot, adc_reading);
    // debug_c_status();  // Commented out to reduce overhead
#endif
    return &current_snapshot;
}

void monitor_dump(const battery_monitor_t *monitor, const char* tag) {
    if (!monitor) return;
    if(!monitor->slow_window) 
        ILOG(TAG, "%s  Monitor: voltage_raw=%hu data_source=%d", tag, monitor->voltage_raw, monitor->monitor_src);
    if(monitor->battery_state) adc_current_state_print(monitor->battery_state, 
        "  Battery State");
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    if(monitor->button_state) adc_current_state_print(monitor->button_state, 
        "   Button State");
#endif
    if(monitor->slow_window) running_avg_print(monitor->slow_window,
        "    Slow Window");
    if(monitor->plateau)  plateau_print(monitor->plateau, 
        "        Plateau");
#if !defined(ULP_MODE) && (C_LOG_LEVEL <= LOG_DEBUG_NUM)
    battery_dump_calibration(&current_calibration, "C Monitor");
#endif
}

void snapshot_dump(const battery_snapshot_t *snapshot, const char* tag) {
    if(!snapshot) return;
    ILOG(TAG, "%s Snapshot: snapshot_timestamp: %lu, snapshot_valid: %d",
         tag, snapshot->snapshot_timestamp, snapshot->snapshot_valid);
    monitor_dump(&snapshot->battery_monitor, tag);
}

battery_monitor_t * battery_get_monitor_handle(void) {
    return &current_snapshot.battery_monitor;
}

enum adc_state_t {
    STATE_CURRENT = 0,
    STATE_LAST = 1,
    STATE_EVENT_PENDING = 2
};

static uint8_t battery_get_state(const uint8_t src, const uint8_t state_type) {
    uint8_t state = 0;
#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)
    const battery_monitor_t* monitor = c_live_snap_get_monitor();
#else
    const battery_monitor_t* monitor = ulp_live_snap_get_monitor();
#endif
    if (!monitor) return state;
    const adc_current_state_t* cs = 
#if defined(CONFIG_ULP_BUTTON_ENABLED)
        (src == 1) ? monitor->button_state : 
#endif
        monitor->battery_state;
    if (!cs) return state;
    switch (state_type) {
        case STATE_CURRENT: state = cs->curr; break;
        case STATE_LAST: state = cs->last; break;
        case STATE_EVENT_PENDING: state = cs->event_pending; break;
    }
    return state;
}

inline uint8_t battery_get_current_battery_state(void) {
    return battery_get_state(0, STATE_CURRENT);
}

inline uint8_t battery_get_last_battery_state(void) {
    return battery_get_state(0, STATE_LAST);
}

inline uint8_t battery_get_pending_battery_event(void) {
    return battery_get_state(0, STATE_EVENT_PENDING);
}

#if defined(CONFIG_ULP_BUTTON_ENABLED)
inline uint8_t battery_get_current_button_state(void) {
    return battery_get_state(1, STATE_CURRENT);
}

inline uint8_t battery_get_last_button_state(void) {
    return battery_get_state(1, STATE_LAST);
}

inline uint8_t battery_get_pending_button_event(void) {
    return battery_get_state(1, STATE_EVENT_PENDING);
}
#endif

uint8_t battery_set_battery_state(battery_monitor_t* monitor, adc_battery_state_t new_state, bool update_last) {
    if (!monitor || !monitor->battery_state || monitor->battery_state->curr == new_state) return 0;
    FUNC_ENTRY_ARGSD(TAG, "new_state=%d", new_state);
    if (update_last) monitor->battery_state->last = monitor->battery_state->curr;
    monitor->battery_state->curr = new_state;
    return 1;
}


#ifndef ULP_MODE
battery_monitor_t* battery_get_monitor_handle(void) {
    return &c_monitor;
}
#endif

inline uint8_t adc_is_charging() {
    return battery_get_current_battery_state() == ADC_BATTERY_CHARGING ? 1 : 0;
}
