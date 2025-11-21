/**
 * ESP32 ulp assembly file for FSM-based ADC monitoring.
 * Does low-power ADC measurements and battery state management.
 */
#include "ulp_config.h"

#if CONFIG_ULP_BUTTON_ENABLED
.set BUTTON_RTC_IO, ULP_BUTTON_RTC_IO
.set LONG_PRESS_THRESHOLD, ULP_LONG_PRESS_CYCLES
#endif

.set HISTORY_SIZE, ULP_ADC_HISTORY_SIZE
.set HISTORY_SHIFT, ULP_ADC_HISTORY_SHIFT
.set STABILIZATION_DELAY, ULP_ADC_STABILIZATION_DELAY
.set adc_channel, ULP_ADC_CHANNEL
.set adc_oversampling_factor_log, ULP_ADC_OVERSAMPLING
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)
.set WAIT_TICKS_PER_ITER, ULP_WAIT_TICKS_PER_ITER
.set WAIT_ITERATIONS, ULP_WAIT_ITERATIONS

/* Battery state constants */
.set BAT_STATE_NORMAL, ADC_ULP_BAT_STATE_NORMAL
.set BAT_STATE_CRITICAL_LOW, ADC_ULP_BAT_STATE_CRITICAL_LOW
.set BAT_STATE_CHARGING_STARTED, ADC_ULP_BAT_STATE_CHARGING_STARTED
.set BAT_STATE_CHARGING_STOPPED, ADC_ULP_BAT_STATE_CHARGING_STOPPED

/* ADP Constants */
.set ADP_DELTA_MIN, 50

/* Delta min constants */
.set DELTA_RISE_CRIT, DELTA_MIN_RISE_CRITICAL
.set DELTA_RISE_FULL, DELTA_MIN_RISE_FULL
.set DELTA_FALL_CRIT, DELTA_MIN_FALL_CRITICAL
.set DELTA_FALL_FULL, DELTA_MIN_FALL_FULL
    
/* Directional threshold constants */
.set RISE_CRIT, RISE_THRESH_CRITICAL
.set RISE_DIS, RISE_THRESH_DISCHARGING
.set RISE_NOM, RISE_THRESH_NOMINAL
.set RISE_CHG, RISE_THRESH_CHARGING
.set RISE_FULL, RISE_THRESH_FULL

.set FALL_CRIT, FALL_THRESH_CRITICAL
.set FALL_DIS, FALL_THRESH_DISCHARGING
.set FALL_NOM, FALL_THRESH_NOMINAL
.set FALL_CHG, FALL_THRESH_CHARGING
.set FALL_FULL, FALL_THRESH_FULL

/* Temporary registers for ULP (using only R0-R3) */
.bss
/* Battery state management */
.global curr_battery_state
curr_battery_state: .long 0
.global last_battery_state
last_battery_state: .long 0

/* Delta min calculation */
.global temp_delta_min
temp_delta_min: .long 0
.global delta_abs
delta_abs: .long 0

/* ADP variables - matching C structure */
.global plateau_last_sample
plateau_last_sample: .long 0
.global plateau_direction
plateau_direction: .long 0
.global plateau_count
plateau_count: .long 0
.global plateau_delta
plateau_delta: .long 0
.global plateau_delta_sum
plateau_delta_sum: .long 0
.global plateau_delta_avg
plateau_delta_avg: .long 0
.global plateau_adaptive_threshold
plateau_adaptive_threshold: .long 0
.global plateau_samples_needed
plateau_samples_needed: .long 0

.global plateau_processing
plateau_processing: .long 0

.global calibrated_voltage_3V2
calibrated_voltage_3V2: .long 0
.global calibrated_voltage_3V6
calibrated_voltage_3V6: .long 0
.global calibrated_voltage_3V8
calibrated_voltage_3V8: .long 0
.global calibrated_voltage_4V0
calibrated_voltage_4V0: .long 0
.global calibrated_voltage_4V1
calibrated_voltage_4V1: .long 0
.global calibrated_voltage_4V2
calibrated_voltage_4V2: .long 0

.global charging_active
charging_active: .long 0

#ifndef SNAPSHORT_AS_ARRAY
.global snapshot_voltage
snapshot_voltage: .long 0
.global snapshot_timestamp
snapshot_timestamp: .long 0
.global snapshot_valid
snapshot_valid: .long 0
#else
.global snapshot
snapshot: .skip ULP_SNAPSHOT_RAW_FIELDS_NUM * 4
#endif

/* History buffer */
.global slow_samples
slow_samples: .skip HISTORY_SIZE * 4
.global slow_idx
slow_idx: .long 0
.global slow_count
slow_count: .long 0
.global slow_sum
slow_sum: .long 0
.global slow_avg
slow_avg: .long 0
.global last_result
last_result: .long 0
.global cycle_count
cycle_count: .long 0
.global main_cpu_running
main_cpu_running: .long 0

/* Debug variables */
#if DEBUG_ULP_VALUES
.global debug_adp_delta_path
debug_adp_delta_path: .long 0
.global debug_delta_min
debug_delta_min: .long 0
#endif

/* Button variables */
#if CONFIG_ULP_BUTTON_ENABLED
.global button_press_counter
button_press_counter: .long 0
.global button_last_result
button_last_result: .long 0
.global curr_button_state
curr_button_state: .long 0
.global last_button_state
last_button_state: .long 0
#endif

.text

/* ========================================================================== */
/* MAIN PROGRAM FLOW */
/* ========================================================================== */

.global entry
entry:
    move r1, cycle_count
    ld r0, r1, 0
    move r2, 0
    sub r2, r0, r2
    jump first_run_init, eq
    jump sleep_cycle

first_run_init:
    move r1, plateau_adaptive_threshold
    move r0, ADP_DELTA_MIN
    st r0, r1, 0
    jump wake_check

sleep_cycle:
    stage_rst
    stage_inc WAIT_ITERATIONS
wait_loop:
    wait WAIT_TICKS_PER_ITER
    stage_dec 1
    jumps wait_loop, 1, ge
    jump wake_check

wake_check:
    jump process_adc

process_adc:
    move r1, cycle_count
    ld r2, r1, 0
    add r2, r2, 1
    st r2, r1, 0
    move r3, HISTORY_SIZE
    sub r3, r2, r3
    jump slow_count_full, ov
    move r2, HISTORY_SIZE
slow_count_full:
    move r1, slow_count
    st r2, r1, 0
    jump adc_measure

/* ========================================================================== */
/* ADC MEASUREMENT */
/* ========================================================================== */

.set ABS_MIN_VALUE_ALLOWED, 400
.set ABS_MAX_VALUE_ALLOWED, 3700

adc_measure:
    move r0, STABILIZATION_DELAY
adc_delay:
    sub r0, r0, 1
    jump adc_delay, ov
    move r0, 0
    stage_rst
adc_read_loop:
    adc r1, 0, adc_channel + 1
    add r0, r0, r1
    stage_inc 1
    jumps adc_read_loop, adc_oversampling_factor, lt
    rsh r0, r0, adc_oversampling_factor_log
    and r0, r0, 0xFFF

    /* Range check */
    move r2, r0
    move r1, ABS_MIN_VALUE_ALLOWED
    sub r1, r2, r1
    jump adc_error, ov
    move r1, ABS_MAX_VALUE_ALLOWED  
    sub r1, r1, r2
    jump adc_error, ov
    move r0, r2
    jump adc_valid

adc_valid:
    move r1, last_result
    st r0, r1, 0
    jump adc_buffer_update

adc_error:
    jump sleep_cycle

adc_buffer_update:
    /* Update circular buffer */
    move r1, slow_idx
    ld r2, r1, 0
    move r1, slow_samples
    add r1, r1, r2
    ld r0, r1, 0
    move r1, slow_sum
    ld r3, r1, 0
    sub r3, r3, r0
    st r3, r1, 0
    move r1, last_result
    ld r0, r1, 0
    move r1, slow_sum
    ld r3, r1, 0
    add r3, r3, r0
    st r3, r1, 0
    move r1, slow_idx
    ld r2, r1, 0
    move r3, slow_samples
    add r3, r3, r2
    move r1, last_result
    ld r0, r1, 0
    st r0, r3, 0
    move r1, slow_idx
    ld r2, r1, 0
    add r2, r2, 1
    move r3, (HISTORY_SIZE-1)
    and r2, r2, r3
    st r2, r1, 0

adc_buffer_check:
    move r1, cycle_count
    ld r2, r1, 0
    move r3, HISTORY_SIZE
    sub r0, r2, r3
    jump sleep_cycle, ov
    move r1, slow_sum
    ld r2, r1, 0
    rsh r2, r2, HISTORY_SHIFT
    move r1, slow_avg
    st r2, r1, 0 // r2 = slow_avg
    jump check_state_transitions

/* ========================================================================== */
/* FAST VOLTAGE CHANGES */
/* ========================================================================== */

check_state_transitions:
    /* === DIRECTION CALCULATION === */
    move r3, last_result
    ld r3, r3, 0    // r3 = last result
    sub r1, r3, r2  // last_result - slow_avg
    move r0, plateau_direction
    st r1, r0, 0    // r1 = direction

    // slow_avg is r2
    move r3, calibrated_voltage_4V2
    ld r3, r3, 0
    sub r3, r3, r2 // 4V2 - slow_avg 
    jump fast_charging_detected, ov // (i.e., slow_avg > 4V1)

    /* Check critical low */
    move r3, calibrated_voltage_3V2
    ld r3, r3, 0
    sub r3, r2, r3 // slow_avg - 3V2
    jump critical_low_detected, ov // (i.e., 3V2 > slow_avg)
    
    /* Check recovery from critical low */
    move r3, curr_battery_state
    ld r3, r3, 0
    move r0, BAT_STATE_CRITICAL_LOW
    sub r3, r3, r0 // curr_battery_state - BAT_STATE_CRITICAL_LOW
    jump check_recovery, eq
    jump adp_detection

check_recovery:
    /* Recovered from critical low - check voltage */
    /* Only executed when curr_battery_state == BAT_STATE_CRITICAL_LOW */
    /* Recovered from critical low - check voltage with hysteresis */
    move r3, calibrated_voltage_3V2
    ld r3, r3, 0
    add r3, r3, 20  /* Simple hysteresis */
    sub r3, r2, r3 // slow_avg - (3V2 + hysteresis)
    rsh r3, r3, 15 // Check sign bit (1 if negative, 0 if positive)
    sub r3, r3, 1  // Make 0xFFFF -> 0
    jump adp_detection, eq   /* If negative, still below, continue ADP */
#if DEBUG_ULP_VALUES
    move r3, debug_adp_delta_path
    move r0, 5
    st r0, r3, 0
#endif
    move r1, BAT_STATE_NORMAL
    jump initiate_state_transition

critical_low_detected:
#if DEBUG_ULP_VALUES
    move r3, debug_adp_delta_path
    move r0, 6
    st r0, r3, 0
#endif
    move r1, BAT_STATE_CRITICAL_LOW
    jump initiate_state_transition

fast_charging_detected: // applies when slow_avg > 4v2
#if DEBUG_ULP_VALUES
    move r3, debug_adp_delta_path
    move r0, 7
    st r0, r3, 0
#endif
    /* Check if plateau_direction >= 0 */
    rsh r3, r1, 15  /* r3 = sign bit (0 if >=0, 1 if <0) */
    jump proceed_to_charging, eq  /* Jump if plateau_direction >= 0 */
    /* Negative: negate */
    move r3, 0
    sub r3, r3, r1  /* r3 = -plateau_direction */
    /* Now r3 = abs(plateau_direction) */
    sub r3, r3, RISE_THRESH_DISCHARGING  /* abs - RISE_THRESH_DISCHARGING */
    jump proceed_to_charging, ov  /* If ov (abs <= RISE_THRESH_DISCHARGING), proceed to charging */
    jump adp_detection  /* Else (abs > RISE_THRESH_DISCHARGING), jump to adp_detection */

proceed_to_charging:
    move r1, BAT_STATE_CHARGING_STARTED
    jump initiate_state_transition

/* ========================================================================== */
/* ADP DETECTION */
/* ========================================================================== */
/* r2 = current slow avg */
/* r1 = direction (last_result - slow_avg) */
adp_detection:
    /* === FAST CHARGING DETECTION === */
    move r3, plateau_samples_needed
    move r0, calibrated_voltage_3V6
    ld r0, r0, 0
    sub r0, r0, r2 // r0 = 3V6 - slow_avg
    jump is_curr_battery_state_normal, ov

    move r0, PLATEAU_SAMPLES_CRITICAL
    st r0, r3, 0
    move r0, RISE_THRESH_CRITICAL
    jump goto_fast_charging_check

is_curr_battery_state_normal:
    move r0, calibrated_voltage_4V0
    ld r0, r0, 0
    sub r0, r0, r2 // r0 = 4V0 - slow_avg
    jump is_curr_battery_state_near_full, ov
    move r0, PLATEAU_SAMPLES_NOMINAL
    st r0, r3, 0
    jump set_fast_charging_threshold_upper
is_curr_battery_state_near_full:
    move r0, PLATEAU_SAMPLES_NOMINAL
    st r0, r3, 0
set_fast_charging_threshold_upper:
    move r0, RISE_THRESH_MAX // plateau_direction - charge_threshold

goto_fast_charging_check:
    move r2, plateau_adaptive_threshold
    st r0, r2, 0
    move r2, curr_battery_state
    ld r2, r2, 0 // r2 = curr_battery_state
    sub r0, r1, r0  // r0 = plateau_direction - charge_threshold (i.e., r1 - r0)
    rsh r0, r0, 15  // Shift right by 15 to get the sign bit (0 if positive, 1 if negative)
    sub r0, r0, 1   // If sign bit was 1 (negative), this will set r0 to 0; if 0 (positive), r0 becomes -1 (0xFFFF)
    jump adp_direction_filtering, eq  // Jump if r0 == 0 (i.e., original diff was negative, so r1 <= r0)
    jump adp_fast_charging_check      // Otherwise, diff was positive, so r1 > r0

adp_fast_charging_check:
#if DEBUG_ULP_VALUES
    move r0, debug_adp_delta_path
    move r3, 8
    st r3, r0, 0
#endif
    sub r0, r2, BAT_STATE_CHARGING_STARTED // curr_battery_state - BAT_STATE_CHARGING_STARTED
    jump adp_direction_filtering, eq
    jump adp_trigger_charging_start  /* if not charging, trigger */

adp_direction_filtering:
    /* === DIRECTION-BASED FILTERING === */
    /* Only filter if not already processing */
    move r3, plateau_processing
    ld r3, r3, 0              /* r3 = plateau_processing */
    sub r3, r3, 0            /* Check if processing == 0 */
    jump adp_proceed_detection, ov  /* If processing != 0, skip filtering */

    /* if(curr_battery_state == ADC_BATTERY_CRITICAL_LOW) return 1 */
    sub r0, r2, BAT_STATE_CRITICAL_LOW // curr_battery_state - BAT_STATE_CRITICAL_LOW
    jump adp_proceed_detection, eq

    /* if (direction >= 0 && curr_battery_state == ADC_BATTERY_CHARGING) return 0 */
    rsh r0, r1, 15 // Check sign bit (1 if negative, 0 if positive)
    sub r0, r0, 1 // Make 0xFFFF -> 0
    jump check_second_condition, eq // jump when negative
    sub r0, r2, BAT_STATE_CHARGING_STARTED // curr_battery_state - BAT_STATE_CHARGING_STARTED
    jump adp_skip_detection, eq // pos dir and charging
    jump adp_proceed_detection
    
check_second_condition:
    /* if (direction <= 0 && curr_battery_state != ADC_BATTERY_CHARGING) return 0 */
    sub r0, r2, BAT_STATE_CHARGING_STARTED
    jump adp_proceed_detection, eq // jump when charging
    jump adp_skip_detection // neg direction and not charging
    
adp_skip_detection:
    /* Skip detection for this cycle */
    jump adp_update_sample

adp_proceed_detection:
adp_normal_proceed:
    move r2, plateau_last_sample
    ld r2, r2, 0  // r2 = plateau last sample (prev last_result)

    /* First sample check */
    sub r1, r2, 0 /* last - 0 */
    jump adp_first_sample, eq /* If last == 0, first sample */
    
    /* === DELTA CALCULATION === */
    /* Calculate delta = last_result - plateau_last */
    move r3, last_result
    ld r3, r3, 0  // r3 = last sample

    sub r1, r3, r2            /* last_result - plateau_last */
    move r0, plateau_delta
    st r1, r0, 0 // save delta

    /* === ABSOLUTE DELTA === */
    /* Check if delta is negative by comparing last_result < plateau_last */
    sub r1, r3, r2            /* last_result - plateau_last */
    jump adp_calc_abs_negative, ov /* Jump if negative (overflow) */

adp_calc_abs_positive:
    /* Positive delta: abs = last_result - plateau_last */
    jump adp_abs_done

adp_calc_abs_negative:
    /* Negative delta: abs = plateau_last - last_result */
    sub r1, r2, r3 

adp_abs_done:
    move r3, delta_abs
    st r1, r3, 0

    // r2 and r3 can be changed here if needed
    /* === GET DELTA MIN === */
    /* Call get_delta_min, uses globals directly, result in R0 */
    jump get_delta_min  /* Call get_delta_min, result in R0 */
delta_min_done:
#if DEBUG_ULP_VALUES
    move r3, debug_delta_min
    st r0, r3, 0 /* store delta_min for debugging */
#endif

    /* === PLATEAU DETECTION === */
    /* Check if abs_delta <= delta_min (small delta/plateau) */
    move r1, delta_abs
    ld r1, r1, 0              // r2 = abs_delta (absolute value of delta)
    sub r1, r1, r0            // r0 = abs_delta - delta_min
    jump adp_small_delta, ov  // Jump if ov (borrow, i.e., abs_delta < delta_min)
    jump adp_small_delta, eq  // Jump if eq (abs_delta == delta_min)

adp_large_delta:
    /* Large delta - reset counters and set hysteresis */
    move r0, plateau_processing
    move r1, 1
    st r1, r0, 0
    // move r0, plateau_adaptive_threshold
    // move r1, 200
    // st r1, r0, 0

#if DEBUG_ULP_VALUES
    move r3, debug_adp_delta_path
    move r0, 10
    st r0, r3, 0
#endif
    jump adp_reset_variables

adp_small_delta:
    /* Small delta */
    /* check plateau_count < 100 */
    move r1, plateau_count    /* Load the memory address of 'plateau_count' into register r1 */
    ld r0, r1, 0              /* Load the current value at that address (offset 0) into register r0 */
    
    // Check if >= PLATEAU_COUNT_MAX
    sub r2, r0, PLATEAU_COUNT_MAX  // r2 = plateau_count - PLATEAU_COUNT_MAX
    rsh r2, r2, 15  // Check sign bit (1 if negative, 0 if positive)
    sub r2, r2, 0   // Make 0xFFFF -> 0
    jump adp_update_sample, eq  // If r3 == 0 (plateau_count >= PLATEAU_COUNT_MAX), jump skip increment
    /* Increment plateau count */
    add r0, r0, 1
    st r0, r1, 0 /* Store updated plateau_count */

    /* Update delta sum */
    move r1, plateau_delta_sum
    ld r3, r1, 0
    move r2, plateau_delta
    ld r2, r2, 0
    add r2, r2, r3
    st r2, r1, 0 // store updated delta_sum

#if DEBUG_ULP_VALUES
    move r1, debug_adp_delta_path
    move r2, 11
    st r2, r1, 0 /* store plateau_count to r1 */
#endif

    /* Check if plateau confirmed */
    move r1, plateau_samples_needed
    ld r1, r1, 0 /* r1 = samples_needed */
    sub r1, r0, r1 /* plateau_count - plateau_samples_needed */
    jump adp_update_sample, ov /* If plateau_count < plateau_samples_needed, continue sampling */

adp_plateau_confirmed:
    move r2, plateau_processing
    ld r0, r2, 0 /* r2 = processing value */
    sub r0, r0, 1 /* (r2 = processing) - (r1 = 1) */
    jump adp_update_sample, ov /* Not processing when processing != 1 */

    /* Set processing = 0 */
    move r1, 0
    st r1, r2, 0

    /** DIRECTIONAL THRESHOLD CALCULATION **/
    /* Calculate delta_avg = delta_sum / count (count = plateau_count, dynamic division) */
    move r1, plateau_delta_sum
    ld r1, r1, 0 /* r1 = delta_sum */

    /* Check sign of delta_sum */
    rsh r0, r1, 15 /* r0 = sign bit (1 if negative) */
    sub r0, r0, 1 /* if r0 == 1, negative */
    jump delta_sum_positive, ov

    /* Negative: negate */
    move r2, 0
    sub r2, r2, r1 /* r2 = -r1 */
    jump do_division

delta_sum_positive:
    move r2, r1

do_division:
    /* Add rounding */
    add r2, r2, 2

    /* Divide r2 by plateau_count using repeated subtraction */
    move r3, plateau_count
    ld r3, r3, 0  // r3 = count
    move r0, 0  // quotient
div_loop:
    sub r2, r2, r3
    jump div_done, ov
    add r0, r0, 1
    jump div_loop
div_done:
    // move r1, r0  // r1 = quotient

    /* If original was negative, negate r1 back */
    // rsh r0, r1, 15 /* reuse r0 to check original sign */
    // sub r0, r0, 1
    // jump store_delta_avg, ov  // if positive, done
    // move r0, 0
    // sub r0, r0, r1
    // move r1, r0
    // when negative, negate back
    // move r1, 0
    // sub r1, r1, r2 /* r1 = -r2 */
    jump store_delta_avg

store_delta_avg:
    move r3, plateau_delta_avg
    st r1, r3, 0
    
    move r0, slow_avg
    ld r0, r0, 0 /* r0 = avg */
    move r2, plateau_direction
    ld r2, r2, 0 /* r2 = direction */

    /* Check if delta_avg < 10 */
    sub r3, r1, 10 /* delta_avg - 10 */
    jump stable_case, ov /* if delta_avg < 10 */

    /* Not stable: adaptive_threshold = get_directional_threshold(avg, direction) */
    // r2 should be direction!!!
    move r3, 1  // return flag, r3 = 1 for not stable
    jump get_directional_threshold
not_stable_continue:
    jump plateau_confirmed_done

stable_case:
    /* Stable: adaptive_threshold = get_directional_threshold(avg, direction) / 2, clamp to 15 */
    move r3, 0  // return flag, r3 = 0 for stable
    jump get_directional_threshold
stable_continue:
    rsh r1, r0, 1 /* / 2 */
    move r2, 15
    sub r1, r2, r1 /* 15 - threshold */
    jump set_stable_threshold, ov // jump if threshold > 15
    move r0, r2
set_stable_threshold:
plateau_confirmed_done:
#if DEBUG_ULP_VALUES
    move r1, debug_adp_delta_path
    move r2, 12
    st r2, r1, 0
#endif
    move r1, plateau_adaptive_threshold
    st r0, r1, 0

    /* Get current state and direction for state transitions */
    move r1, plateau_direction
    ld r3, r1, 0 /* r0 = direction */
    // move r2, plateau_adaptive_threshold
    // ld r2, r2, 0 /* r2 = adaptive threshold */    

    /* Compute abs(direction) */
    rsh r1, r3, 15 // r1 = sign bit (0 if >=0, 1 if <0)
    sub r1, r1, 1  // check if r1 == 1 (<0)
    jump dir_positive, ov
    move r1, 0
    sub r1, r1, r3 /* r1 = -r3 */
    jump check_adaptive_threshold
dir_positive:
    move r1, r3
check_adaptive_threshold:
    sub r1, r1, r0 /* diff - threshold */
    jump adp_reset_counters, ov /* if diff <= threshold, reset */

    move r1, curr_battery_state
    ld r2, r1, 0 /* r2 = current battery state */

    rsh r1, r3, 15
    sub r1, r1, 1

    jump check_charging_stop, eq /* if diff < 0 */
    jump check_charging_start    /* if diff > 0 */
    
check_charging_start:
    /* charging start only possible if state is NORMAL */
    sub r1, r2, BAT_STATE_NORMAL  /* r1 = curr_state - NORMAL */
    jump adp_trigger_charging_start, eq
    
check_charging_stop:
    /* charging stop only possible if state is CHARGING_STARTED */
    sub r1, r2, BAT_STATE_CHARGING_STARTED  /* r1 = curr_state - CHARGING_STARTED */
    jump adp_trigger_charging_stop, eq
    
    jump adp_reset_counters

adp_trigger_charging_start:
    move r1, BAT_STATE_CHARGING_STARTED
    jump initiate_state_transition

adp_trigger_charging_stop:
    move r1, BAT_STATE_NORMAL
    jump initiate_state_transition

adp_first_sample:
adp_reset_counters:
    move r0, plateau_processing
    st r1, r0, 0
adp_reset_variables:
    /* Reset plateau for next detection */
    move r0, plateau_count
    move r1, 0
    st r1, r0, 0
    move r0, plateau_delta_sum
    st r1, r0, 0

adp_update_sample:
    /* Update last sample */
    move r2, last_result
    ld r2, r2, 0 // load last_result value to r2 from r2
    move r1, plateau_last_sample
    st r2, r1, 0 // store last_result to plateau_last_sample
    jump sleep_cycle

/* ========================================================================== */
/* STATE TRANSITION HANDLING */
/* ========================================================================== */

initiate_state_transition:
    /* Update last sample */
    move r2, last_result
    ld r2, r2, 0 // load last_result value to r2 from r2
    move r3, plateau_last_sample
    st r2, r3, 0 // store last_result to plateau_last_sample
    
    /* r1 = new battery state */
    move r2, curr_battery_state
    ld r0, r2, 0
    sub r0, r1, r0 /* r0 = new_state - current_state */
    jump sleep_cycle, eq /* No change */

    st r1, r2, 0 /* Update current state */

    /* RESET PLATEAU COUNTERS */
    move r0, plateau_count
    move r1, 0
    st r1, r0, 0
    move r0, plateau_processing
    st r1, r0, 0

    jump take_snapshot

take_snapshot:
    move r3, cycle_count
    ld r0, r3, 0
    move r3, snapshot_timestamp
    st r0, r3, 0

    move r3, last_result
    ld r0, r3, 0
    move r3, snapshot_voltage
    st r0, r3, 0

    move r3, snapshot_valid
    move r0, 1
    st r0, r3, 0

    /* Check if main CPU is sleeping */
    move r3, main_cpu_running  
    ld r0, r3, 0
    move r3, 0
    sub r3, r0, r3
    jump sleep_cycle, ov
    jump wake_main_cpu

wake_main_cpu:
    wake
    halt

#if CONFIG_ULP_BUTTON_ENABLED
button_check:
    jump sleep_cycle
#endif

// get_directional_threshold: 
// inputs r0 = avg, 
// r2 = direction; 
// output r0 = threshold
get_directional_threshold:
    // Check direction >= 0 (rising)
    rsh r1, r2, 15  // r1 = sign bit (0 if >=0, 1 if <0)
    sub r1, r1, 1  // check if r1 == 1 (<0)
    jump falling_threshold, eq  // If negative (overflow), falling

    // Rising branch
    move r1, calibrated_voltage_3V6
    ld r1, r1, 0
    sub r1, r1, r0  // avg - 3V6
    jump rising_3V8, ov
    move r0, RISE_CRIT
    jump threshold_done

rising_3V8:
    move r1, calibrated_voltage_3V8
    ld r1, r1, 0
    sub r1, r1, r0
    jump rising_4V0, ov
    move r0, RISE_DIS
    jump threshold_done

rising_4V0:
    move r2, calibrated_voltage_4V0
    ld r2, r2, 0
    sub r2, r2, r0
    jump rising_4V1, ov
    move r0, RISE_NOM
    jump threshold_done

rising_4V1:
    move r2, calibrated_voltage_4V1
    ld r2, r2, 0
    sub r2, r2, r0
    jump rising_full, ov
    move r0, RISE_CHG
    jump threshold_done

rising_full:
    move r0, RISE_FULL
    jump threshold_done

falling_threshold:
    // Falling branch
    move r2, calibrated_voltage_3V6
    ld r2, r2, 0
    sub r2, r2, r0
    jump falling_3V8, ov
    move r0, FALL_CRIT
    jump threshold_done

falling_3V8:
    move r2, calibrated_voltage_3V8
    ld r2, r2, 0
    sub r2, r2, r0
    jump falling_4V0, ov
    move r0, FALL_DIS
    jump threshold_done

falling_4V0:
    move r2, calibrated_voltage_4V0
    ld r2, r2, 0
    sub r2, r2, r0
    jump falling_4V1, ov
    move r0, FALL_NOM
    jump threshold_done

falling_4V1:
    move r2, calibrated_voltage_4V1
    ld r2, r2, 0
    sub r2, r2, r0
    jump falling_full, ov
    move r0, FALL_CHG
    jump threshold_done

falling_full:
    move r0, FALL_FULL

threshold_done:
    sub r3, r3, 1  // Check if 1 (not stable)
    jump not_stable_continue, eq  // If flag == 1, jump to not stable
    jump stable_continue  // Else (flag == 0), jump to stable

// Updated get_delta_min: 
// no inputs needed, uses globals directly;
// r2 = slow_avg
// r3 = direction
// output R0=delta_min
get_delta_min:
    move r2, slow_avg
    ld r2, r2, 0
    // Base calculation (rising branch first)
    move r3, plateau_direction
    ld r3, r3, 0  // r3 = direction

    rsh r1, r3, 15  // r1 = sign bit (0 if >=0, 1 if <0)
    sub r1, r1, 1  // check if r1 == 1 (<0)
    jump falling, eq  // if direction < 0, jump to falling

    // Rising: direction >= 0
    move r1, calibrated_voltage_3V6
    ld r1, r1, 0  // r1 = VOLT_3V6
    sub r1, r2, r1  // voltage - VOLT_3V6
    jump rise_crit, ov  // If voltage < VOLT_3V6 (overflow), critical
    move r1, DELTA_RISE_FULL
    move r0, temp_delta_min
    st r1, r0, 0
    jump apply_rules

rise_crit:
    move r1, DELTA_RISE_CRIT
    move r0, temp_delta_min
    st r1, r0, 0
    jump apply_rules

falling:
    // Falling: direction < 0
    move r1, calibrated_voltage_3V6
    ld r1, r1, 0  // r1 = VOLT_3V6
    sub r1, r2, r1  // voltage - VOLT_3V6
    jump fall_crit, ov  // If voltage < VOLT_3V6 (overflow), critical
    move r1, DELTA_FALL_FULL
    move r0, temp_delta_min
    st r1, r0, 0
    jump apply_rules

fall_crit:
    move r1, DELTA_FALL_CRIT
    move r0, temp_delta_min
    st r1, r0, 0
    jump apply_rules

apply_rules:
    // Rule 1: if (direction > 0 && avg >= VOLT_4V0) delta_min = max(delta_min, 50)
    rsh r1, r3, 15  // r1 = sign bit (0 if >=0, 1 if <0)
    sub r1, r1, 1  // check if r1 == 1 (<0)
    jump rule2, ov
    jump rule1_check_avg  // If positive (no overflow), check avg

rule1_check_avg:
    move r1, calibrated_voltage_4V0
    ld r1, r1, 0  // r1 = VOLT_4V0
    sub r1, r2, r1  // avg - VOLT_4V0
    jump rule1_apply, ov  // If avg >= VOLT_4V0 (no overflow), apply rule
    jump rule2

rule1_apply:
    move r1, temp_delta_min
    ld r0, r1, 0  // r0 = current delta_min
    move r2, 50
    sub r0, r2, r0  // delta_min - 50
    jump rule2, ov  // If delta_min >= 50 (no overflow), skip
    move r0, 50
    st r0, r1, 0

rule2:
    // Rule 2: if (curr_battery_state == BAT_STATE_CHARGING) delta_min = max(delta_min, 25)
    move r0, curr_battery_state
    ld r1, r0, 0  // r1 = curr_battery_state
    sub r1, r1, BAT_STATE_CHARGING_STARTED  // state - BAT_STATE_CHARGING_STARTED
    jump rule2_apply, eq  // If result == 0, equal, apply rule
    jump end  // If result != 0, not equal, skip

rule2_apply:
    move r1, temp_delta_min
    ld r0, r1, 0  // r0 = current delta_min
    move r2, 25
    sub r0, r2, r0  // 25 - delta_min
    jump end, ov  // If delta_min >= 25 (no overflow), skip
    move r0, 25
    st r0, r1, 0

end:
    // Load result into R0
    move r0, temp_delta_min
    ld r0, r0, 0
    jump delta_min_done
