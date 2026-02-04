#include "adc_private.h"
#if defined(CONFIG_LOGGER_ADC_ENABLED)
#include "adc_snapshot.h"
#ifdef ULP_MODE
// #include "adc_ulp.h"
#endif

#define ADC_MIN_VALID 500
#define ADC_MAX_VALID 3500

static const char *TAG = "adc_bat_adp";

#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)

static uint16_t initial_samples[ULP_ADC_HISTORY_SIZE] = {0};
static bool c_live_snap_initialized = false;
static battery_snapshot_t live_snapshot = BATTERY_SNAPSHOT_DEFAULTS();

void c_live_snap_init(void) {
    if(c_live_snap_initialized) return;
    FUNC_ENTRYD(TAG);
//     battery_monitor_init(&live_snapshot.battery_monitor, 
//         &live_plateau, 
//         &live_slow_window, 
//         &live_battery_state,
// #ifdef CONFIG_ULP_BUTTON_ENABLED
//         0
// #endif
//     );
#ifdef ULP_MODE
    // Initially copy values from wakeup state to analyze - intentional for initial state sync
    battery_snapshot_t * ulp_snap = ulp_live_snap_get();
    if (ulp_snap) {
        // Copy slow_window and voltage_raw from ULP for initial analysis
        memcpy(live_snapshot.battery_monitor.slow_window, ulp_snap->battery_monitor.slow_window, sizeof(adc_running_avg_t));
        live_snapshot.battery_monitor.voltage_raw = ulp_snap->battery_monitor.voltage_raw;
        live_snapshot.snapshot_timestamp = ulp_snap->snapshot_timestamp;
        // Initialize C plateau with proper values from ULP slow_window to avoid large initial deltas
        live_snapshot.battery_monitor.plateau->last_sample = ulp_snap->battery_monitor.slow_window->avg;
        live_snapshot.battery_monitor.plateau->adaptive_threshold = get_directional_threshold(ulp_snap->battery_monitor.slow_window->avg, 0);
    }
    // Calibration is already shared via battery_set_calibration/update_ulp_calibration
    // No need to copy from ULP - just ensure it's applied
    if (!is_calibration_applied()) {
        // If calibration not applied yet, it will be applied when battery_set_calibration is called
        DLOG(TAG, "C live snap initialized - calibration will be synced when applied");
    }
#endif
    c_live_snap_initialized = true;
}

battery_snapshot_t * c_live_snap_get(void) {
    if(!c_live_snap_initialized) return 0;
    return &live_snapshot;
}

battery_monitor_t * c_live_snap_get_monitor(void) {
    if(!c_live_snap_initialized) return 0;
    return &live_snapshot.battery_monitor;
}
#endif

void battery_monitor_init(battery_monitor_t* monitor, 
adc_plateau_t * battery_plateau, 
adc_running_avg_t * battery_slow_window, 
adc_current_state_t * adc_current_battery_state
#ifdef CONFIG_ULP_BUTTON_ENABLED
, adc_current_state_t * adc_current_button_state
#endif
) {
    if(!monitor) return;
    FUNC_ENTRYD(TAG);
    // Initialize all state to zero
    // memset(monitor, 0, sizeof(battery_monitor_t));
    if(battery_plateau)
        monitor->plateau = battery_plateau;
    if(battery_slow_window)
        monitor->slow_window = battery_slow_window;
    // running_avg_init(monitor->slow_window, slow_window_buffer, ULP_ADC_HISTORY_SIZE);
    if(adc_current_battery_state)
        monitor->battery_state = adc_current_battery_state;
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    if (adc_current_button_state) {
        monitor->button_state = adc_current_button_state;
    }
#endif
}

#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)

battery_snapshot_t * c_live_snap_update(uint16_t adc_reading) {
    if(!c_live_snap_initialized) return NULL;
#if defined(ULP_MODE)
    battery_snapshot_t * snap = ulp_live_snap_get();
    if(!snap) return NULL;
    if(snap->snapshot_timestamp == live_snapshot.snapshot_timestamp) {
        goto done;
    }
    adc_reading = snap->battery_monitor.voltage_raw;
    live_snapshot.battery_monitor.voltage_raw = adc_reading;
    memcpy(live_snapshot.battery_monitor.slow_window, snap->battery_monitor.slow_window, sizeof(adc_running_avg_t));
    // Initialize C plateau with current values from ULP slow_window to maintain state across deep sleep
    // live_snapshot.battery_monitor.plateau->last_sample = snap->battery_monitor.slow_window->avg;
    // live_snapshot.battery_monitor.plateau->adaptive_threshold = get_directional_threshold(snap->battery_monitor.slow_window->avg, 0);
#endif
    FUNC_ENTRY_ARGSD(TAG, "adc_reading=%" PRIu16 "", adc_reading);
    // Update C monitor (handles state changes, batulp_battery_event_pending, and set_battery_status internally)
    // Update the authoritative C monitor (performs detection and updates its internal state)
    uint8_t prev_state  = live_snapshot.battery_monitor.battery_state->curr;
    adc_battery_state_t new_state = battery_monitor_update(&live_snapshot.battery_monitor, adc_reading);

    // Only report actual battery events, not NORMAL state changes
    int should_report = new_state != prev_state && (new_state == ADC_BATTERY_CHARGING || 
                        new_state == ADC_BATTERY_CHARGING_STOPPED || 
                        new_state == ADC_BATTERY_CRITICAL_LOW);

    if( should_report)
        DLOG(TAG, "New battery state: %s, should_report=%d", adc_battery_states_str(new_state), should_report);

    // NOTE: battery_monitor_update already updates battery_monitor->battery_state->curr
    // and battery_monitor.last_battery_state when a real state change occurs. Do not
    // duplicate or overwrite those fields here. Instead, copy the authoritative
    // monitor into the snapshot for reporting.
#if defined(ULP_MODE)
    live_snapshot.snapshot_timestamp = ulp_live_snap_get()->snapshot_timestamp;
#else
    live_snapshot.snapshot_timestamp = esp_timer_get_time();
#endif
    live_snapshot.snapshot_valid = 1; // Mark snapshot as valid
    done:
    snapshot_dump(c_live_snap_get(), "C Live");
    return &live_snapshot;
}

void running_avg_init(adc_running_avg_t * avg, uint16_t * buffer, uint16_t size) {
    if(!avg) return;
    FUNC_ENTRYD(TAG);
    avg->samples = buffer;
    avg->size = size >= 16 ? 16 : size >= 8 ? 8 : size >= 4 ? 4 : 2;
    avg->shift = avg->size == 16 ? 4 : avg->size == 8 ? 3 : avg->size == 4 ? 2 : 1;
    avg->idx = 0;
    avg->count = 0;
    avg->sum = 0;
    avg->avg = 0;
}

void running_avg_update(adc_running_avg_t * avg, uint16_t new_sample) {
    if(!avg || !avg->samples) return;
    FUNC_ENTRYD(TAG);
    // Remove oldest sample
    uint16_t oldest = avg->samples[avg->idx];
    avg->sum -= oldest;

    // Add new sample
    avg->samples[avg->idx] = new_sample;
    avg->sum += new_sample;

    // Calculate average
    if (avg->count >= avg->size) {
        avg->avg = avg->sum >> avg->shift; // Divide by 8
    } else {
        avg->count++;
        avg->avg = avg->sum / avg->count;
    }
    // Update index
    avg->idx = (avg->idx + 1) % avg->size;
}

inline void update_slow_window(battery_monitor_t* monitor, uint16_t new_sample) {
    running_avg_update(monitor->slow_window, new_sample);
}

#endif

inline uint8_t running_avg_idx(const adc_running_avg_t * avg) {
    return (avg->count == 0) ? 0 : (avg->idx - 1 + avg->size) % avg->size;
}

void running_avg_print(const adc_running_avg_t * avg, const char* name) {
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
    if(!avg) return;
    char buf[128] = {0};
    char *p = buf;
    if (avg->samples != NULL) {
        for (int i = 0, j = avg->count < avg->size ? avg->count : avg->size, k = running_avg_idx(avg); i < j; i++) {
            p += sprintf(p, "%" PRIu16 "%s", avg->samples[i], (i == k) ? "<" : "");
            if (i < j - 1) p += sprintf(p, ",");
        }
    }
    ILOG(TAG, "%s: [%s] sum=%" PRIu32 " avg=%" PRIu16 " count=%" PRIu16 " size=%" PRIu16 " shift=%" PRIu16 " idx=%" PRIu16 "", 
        name, buf, avg->sum, avg->avg, avg->count, avg->size, avg->shift, avg->idx);
#endif
}

/* Helper: compute Mean Absolute Deviation (MAD) of ULP history (raw units) */
uint32_t running_avg_mad(const adc_running_avg_t * avg) {
    uint32_t history_avg = 0, mad = 0, v;
    uint16_t i, available = avg->count < avg->size ? avg->count : avg->size;
    for (i = 0; i < available; i++) {
        history_avg += avg->samples[i];
    }
    history_avg /= available;
    for (i = 0; i < available; i++) {
        v = avg->samples[i];
        mad += (v > history_avg) ? (v - history_avg) : (history_avg - v);
    }
    mad /= available;
    return mad;
}

void adc_current_state_print(const adc_current_state_t* state, const char* name) {
    if(!state) return;
    ILOG(TAG, "%s: curr=%" PRIu8 " last=%" PRIu8 " event_pending=%" PRIu8 "", name,
         state->curr,
         state->last,
         state->event_pending
         );
}

void plateau_print(const adc_plateau_t* plateau, const char* name) {
    if(!plateau) return;
    ILOG(TAG, 
    "%s: count=%" PRIu16 " last_sample=%" PRIu16 " (|direction=%hd| > adaptive_threshold=%" PRIu16 ") "
    "delta=%hd delta_sum=%hd delta_avg=%hd reported=%" PRIu8 " processing=%" PRIu16 ""
         , name,
         plateau->count,
         plateau->last_sample,
         plateau->direction,
         plateau->adaptive_threshold,
         plateau->delta,
         plateau->delta_sum,
         plateau->delta_avg,
         plateau->reported,
         plateau->processing
         );
}

#if defined(USE_REF_SNAPSHOT) || !defined(ULP_MODE)

uint16_t get_directional_threshold(uint16_t voltage, int16_t direction) {
    if (direction >= 0) {
        // Rising voltage thresholds
        if (voltage < current_calibration.voltage_3V6) {
            DLOG(TAG, " =* Rising threshold: critical (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_3V6, RISE_THRESH_CRITICAL);
            return RISE_THRESH_CRITICAL;
        }
        if (voltage < current_calibration.voltage_3V8) {
            DLOG(TAG, " =* Rising threshold: discharging (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_3V8, RISE_THRESH_DISCHARGING);
            return RISE_THRESH_DISCHARGING;
        }
        if (voltage < current_calibration.voltage_4V0) {
            DLOG(TAG, " =* Rising threshold: nominal (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V0, RISE_THRESH_NOMINAL);
            return RISE_THRESH_NOMINAL;
        }
        if (voltage < current_calibration.voltage_4V1) {
            DLOG(TAG, " =* Rising threshold: charging (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V1, RISE_THRESH_CHARGING);
            return RISE_THRESH_CHARGING;
        }
        DLOG(TAG, " =* Rising threshold: full (%" PRIu16 " >= %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V1, RISE_THRESH_FULL);
        return RISE_THRESH_FULL;
    } else {
        // Falling voltage thresholds
        if (voltage < current_calibration.voltage_3V6) {
            DLOG(TAG, " =* Falling threshold: critical (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_3V6, FALL_THRESH_CRITICAL);
            return FALL_THRESH_CRITICAL;
        }
        if (voltage < current_calibration.voltage_3V8) {
            DLOG(TAG, " =* Falling threshold: discharging (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_3V8, FALL_THRESH_DISCHARGING);
            return FALL_THRESH_DISCHARGING;
        }
        if (voltage < current_calibration.voltage_4V0) {
            DLOG(TAG, " =* Falling threshold: nominal (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V0, FALL_THRESH_NOMINAL);
            return FALL_THRESH_NOMINAL;
        }
        if (voltage < current_calibration.voltage_4V1) {
            DLOG(TAG, " =* Falling threshold: charging (%" PRIu16 " < %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V1, FALL_THRESH_CHARGING);
            return FALL_THRESH_CHARGING;
        }
        DLOG(TAG, " =* Falling threshold: full (%" PRIu16 " >= %" PRIu16 "), returning %" PRIu16 "", voltage, current_calibration.voltage_4V1, FALL_THRESH_FULL);
        return FALL_THRESH_FULL;
    }
}

static inline uint8_t get_plateau_samples_needed(uint16_t voltage) {
    if (voltage < current_calibration.voltage_3V6) return PLATEAU_SAMPLES_CRITICAL;
    if (voltage >= current_calibration.voltage_4V0) return PLATEAU_SAMPLES_CHARGING;
    return PLATEAU_SAMPLES_NOMINAL;
}

uint16_t get_delta_min(uint16_t voltage, int16_t direction) {
    if (direction >= 0) {
        // Rising deltas
        if (voltage < current_calibration.voltage_3V6) return DELTA_MIN_RISE_CRITICAL;
        return DELTA_MIN_RISE_FULL;
    } else {
        // Falling deltas
        if (voltage < current_calibration.voltage_3V6) return DELTA_MIN_FALL_CRITICAL;
        return DELTA_MIN_FALL_FULL;
    }
}

inline static uint8_t should_continue_detection(adc_battery_state_t curr_battery_state, int16_t direction) {
    if(curr_battery_state == ADC_BATTERY_CRITICAL_LOW) return 1; // pass through in critical low
    if (direction >= 0 && curr_battery_state == ADC_BATTERY_CHARGING) return 0;
    if (direction <= 0 && curr_battery_state != ADC_BATTERY_CHARGING) return 0;
    return 1;
}

static uint8_t adp_plateau_detection(adc_plateau_t* plateau, uint16_t avg, uint16_t new_sample, int16_t difference, int curr_battery_state) {
    if(!plateau) return 0;
    FUNC_ENTRY_ARGS(TAG, "avg=%" PRIu16 " new_sample=%" PRIu16 " curr_battery_state=%d", avg, new_sample, curr_battery_state);
    plateau_print(plateau, "Plateau Start State");
    uint8_t ret = 0;
    plateau->direction = difference;
    // Direction-based filtering only if not already processing
    if (plateau->processing == 0) {
        if (!should_continue_detection(curr_battery_state, plateau->direction)) {
            DLOG(TAG, "ADP: Direction %hd not allowed in current state %d, skipping detection", plateau->direction, curr_battery_state);
            goto done;
        }
        // Start processing for allowed direction
        DLOG(TAG, "ADP: Starting plateau processing (direction=%hd)", plateau->direction);
    }
    uint8_t plateau_samples_needed = get_plateau_samples_needed(avg);
    plateau->delta = (int16_t)new_sample - (int16_t)plateau->last_sample;
    uint16_t delta_abs = (uint16_t)(plateau->delta < 0 ? -plateau->delta : plateau->delta);
    uint16_t delta_min = get_delta_min(avg, plateau->direction);
    // For charging detection (rising voltage), be more tolerant of large initial jumps
    if (plateau->direction > 0 && avg >= current_calibration.voltage_4V0) {
        delta_min = (delta_min > 50) ? delta_min : 50;  // Allow larger jumps when charging starts
    }

    // During active charging, be more tolerant of voltage fluctuations
    if (curr_battery_state == ADC_BATTERY_CHARGING) {
        // Increase delta_min during charging to avoid false plateau resets from normal fluctuations
        delta_min = (delta_min > 25) ? delta_min : 25;  // At least 25 ADC units tolerance during charging
        DLOG(TAG, "ADP: Increased delta_min to %" PRIu16 " during charging", delta_min);
    }

    if (delta_abs < delta_min) {
        // Potential plateau
        if (plateau->count < PLATEAU_COUNT_MAX) {
            plateau->count++;
        }
        plateau->delta_sum += plateau->delta;
        DLOG(TAG, "ADP: Potential plateau detected, wait: (delta=%d < delta_min=%" PRIu16 ") plateau_samples_needed=%" PRIu16 "", 
            plateau->delta, delta_min, plateau_samples_needed);
        if (plateau->count >= plateau_samples_needed && plateau->processing) {
            // Plateau confirmed - set adaptive threshold based on stability
            plateau->delta_avg = plateau->delta_sum / plateau->count;
            // More stable plateau = lower threshold (more sensitive to changes)
            if (plateau->delta_avg < 10) {
                // Very stable - use aggressive threshold
                plateau->adaptive_threshold = get_directional_threshold(avg, plateau->direction) / 2;
                if (plateau->adaptive_threshold < 13) plateau->adaptive_threshold = 13; 
            } else {
                // Less stable - use normal threshold
                plateau->adaptive_threshold = get_directional_threshold(avg, plateau->direction);
            }
            plateau->processing = 0;  // End processing on reset
            plateau->reported = 1; // Mark as reported
            ret = 1; // Plateau confirmed - signal event
            DLOG(TAG, "!!! ADP: Plateau confirmed, processing done:  delta_sum=%d delta_avg=%d plateau_samples_needed=%" PRIu16 " adaptive_threshold=%" PRIu16 "", 
                plateau->delta_sum, plateau->delta_avg, plateau_samples_needed, plateau->adaptive_threshold);
            goto reset;
        }
    } else {
        if(!plateau->processing) {
            plateau->processing = 1; // Mark that we are now processing a plateau
        }
        plateau->reported = 0;  // Clear reported flag for next plateau
        // Not in plateau - reset
        DLOG(TAG, "ADP: Peak detected, processing started (delta=%" PRIu16 " >= delta_min=%" PRIu16 ")", delta_abs, delta_min);
        // Hysteresis: Set higher threshold after change to prevent immediate re-detection
        // plateau->adaptive_threshold = 200;  // High threshold to avoid repeat detections
        reset:
        plateau->count = 0;
        plateau->delta_sum = 0;
    }
    done:
    plateau->last_sample = new_sample;
    // plateau_print(plateau, "Plateau End State");
    return ret; // No plateau
}

static adc_battery_state_t check_fast_voltage_changes(adc_plateau_t* plateau, uint16_t avg, int16_t difference, adc_battery_state_t curr_battery_state) {
    adc_battery_state_t result_state = curr_battery_state;  // Track the result
    
    if (avg > current_calibration.voltage_4V2) {
        if (curr_battery_state != ADC_BATTERY_CHARGING) {
            DLOG(TAG, "ADP: charging detected (avg=%" PRIu16 ")", avg);
            result_state = ADC_BATTERY_CHARGING;
        }
    } else if (avg <= current_calibration.voltage_3V2) {
        if (curr_battery_state != ADC_BATTERY_CRITICAL_LOW) {
            DLOG(TAG, "ADP: Critical low battery detected (avg=%" PRIu16 ")", avg);
            result_state = ADC_BATTERY_CRITICAL_LOW;
        }
    } else {
        uint16_t charge_threshold = (avg < current_calibration.voltage_3V6) ? 100 : 200;
        if (difference > charge_threshold && curr_battery_state != ADC_BATTERY_CHARGING) {
            DLOG(TAG, "ADP: Charging recovery detected (diff=%hd > threshold=%" PRIu16 ") from low voltage", difference, charge_threshold);
            result_state = ADC_BATTERY_CHARGING;
        }
        if (curr_battery_state == ADC_BATTERY_CRITICAL_LOW) {
            DLOG(TAG, "ADP: Battery recovered from critical low to NORMAL (avg=%" PRIu16 ")", avg);
            result_state = ADC_BATTERY_NORMAL;
        }
    }
    
    if (result_state != curr_battery_state) {
        plateau->processing = 0;  // Reset processing only on actual change
    }
    return result_state;
}

static adc_battery_state_t interpret_plateau_event(adc_plateau_t* plateau, uint16_t avg, uint16_t new_sample, int16_t difference, adc_battery_state_t curr_battery_state) {
    // Calculate delta from plateau last sample (compares to adaptive_threshold calibrated for deltas)
    DLOG(TAG, "ADP: Plateau stable, evaluating diff check (delta=%hd) difference=%hd > adaptive_threshold=%" PRIu16 "", plateau->delta, difference, plateau->adaptive_threshold);
    uint16_t difference_abs = difference >= 0 ? difference : -difference;
    // Check if delta exceeds adaptive threshold (threshold calibrated for deltas, not avg differences)
    if (difference_abs > plateau->adaptive_threshold) {
        DLOG(TAG, "ADP: Significant delta detected (difference_abs=%hd > threshold=%" PRIu16 ")", difference_abs, plateau->adaptive_threshold);
        if(avg > current_calibration.voltage_3V8 || difference_abs > 50)
            curr_battery_state = (difference > 0) ? ADC_BATTERY_CHARGING : ADC_BATTERY_NORMAL;
    }
    return curr_battery_state;
}

static void update_charging_state(battery_monitor_t* monitor, adc_battery_state_t new_state) {
    FUNC_ENTRY_ARGSD(TAG, "new_state=%" PRIu8 "", new_state);
    uint8_t updated = battery_set_battery_state(monitor, new_state, false);
    if (!updated) {
        DLOG(TAG, "ADP: No state change (already in state %" PRIu8 ")", new_state);
        return; // No change
    }
    switch (new_state) {
        case ADC_BATTERY_CHARGING:
            monitor->battery_state->event_pending = 1;
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
        case ADC_BATTERY_NORMAL:
            if (monitor->battery_state->curr == ADC_BATTERY_CHARGING) {
                monitor->battery_state->event_pending = 1;
            }
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            monitor->battery_state->event_pending = 1;
            break;
        default:
            break;
    }
}

uint8_t battery_monitor_update(battery_monitor_t* monitor, uint16_t adc_reading) {
    if(!monitor || !monitor->slow_window || !monitor->battery_state || !monitor->plateau) return ADC_BATTERY_NORMAL;
    FUNC_ENTRY_ARGSD(TAG, "adc_reading=%" PRIu16 "", adc_reading);
#ifndef ULP_MODE
    update_slow_window(monitor, adc_reading);
#endif
    // Validate ADC reading
    if (monitor->slow_window->count < monitor->slow_window->size) {
        return ADC_BATTERY_NORMAL;
    }
    // Cache frequently accessed values
    uint8_t curr_battery_state = monitor->battery_state->curr;
    int16_t difference = (int16_t)adc_reading - (int16_t)monitor->slow_window->avg;
    // Phase 1: Fast voltage-based detection
    adc_battery_state_t fast_state = check_fast_voltage_changes(monitor->plateau, monitor->slow_window->avg, difference, curr_battery_state);
    if (fast_state != curr_battery_state) {
        update_charging_state(monitor, fast_state);
        return fast_state;
    }

    // Phase 2: ADP plateau detection
    if (adp_plateau_detection(monitor->plateau, monitor->slow_window->avg, adc_reading, difference, curr_battery_state)) {
        // Phase 3: Event interpretation (only if plateau confirmed)
        adc_battery_state_t new_state = interpret_plateau_event(monitor->plateau, monitor->slow_window->avg, adc_reading, difference, curr_battery_state);
        if (new_state != curr_battery_state) {
            update_charging_state(monitor, new_state);
            DLOG(TAG, "ADP: state change detected: last_state=%d new_state=%" PRIu8 " -> monitor: curr_battery_state=%" PRIu8 "",
            curr_battery_state, new_state, monitor->battery_state->curr);
            // Handle CHARGING_STARTED -> NORMAL transition as CHARGING_STOPPED event
            if (new_state == ADC_BATTERY_NORMAL && curr_battery_state == ADC_BATTERY_CHARGING) {
                DLOG(TAG, "ADP: Interpreting CHARGING_STARTED -> NORMAL as CHARGING_STOPPED event");
                return ADC_BATTERY_CHARGING_STOPPED;  // Signal stopped event but set state to NORMAL
            }
            return new_state; // Return event state for notification
        }
        else {
            DLOG(TAG, "ADP: No state change detected: curr_battery_state=%" PRIu8 " new_state=%" PRIu8 "", curr_battery_state, new_state);
        }
        
    }
    return curr_battery_state; // No state change
}

#endif 

#endif  /* CONFIG_LOGGER_ADC_ENABLED */