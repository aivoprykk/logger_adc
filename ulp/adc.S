/*
 adc.S - REORGANIZED VERSION
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "../ulp_adc_config.h"
.set CONFIRMATION_READINGS, ULP_CONFIRMATION_READINGS
.set ULP_ADC_CONFIRMATION_SAMPLES, CONFIRMATION_READINGS
.set HISTORY_INDEX_ADDR, 0
/* Configuration from preprocessor flags */
#if CONFIG_ULP_BUTTON_ENABLED
.set BUTTON_RTC_IO, ULP_BUTTON_RTC_IO
.set LONG_PRESS_THRESHOLD, ULP_LONG_PRESS_CYCLES
#endif

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.set ADC_HISTORY_SIZE, ULP_ADC_HISTORY_SIZE
#endif

.set ADC_STABILIZATION_DELAY, ULP_ADC_STABILIZATION_DELAY
.set adc_channel, ULP_ADC_CHANNEL
.set adc_oversampling_factor_log, ULP_ADC_OVERSAMPLING
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

/* Use pre-calculated values
 * NOTE: the preprocessor may expand symbol names on the left-hand side of
 * ".set NAME, VALUE" which breaks the ULP assembler if NAME itself is a
 * macro. Define constant names that are not preprocessor macros and map
 * them to the preprocessor values on the right-hand side. Use the *_CONST
 * identifiers in the ULP code below.
 */
.set WAIT_TICKS_PER_ITER, ULP_WAIT_TICKS_PER_ITER
.set WAIT_ITERATIONS, ULP_WAIT_ITERATIONS

/* Compile-time constants - no RAM needed! */
.set ADC_RAPID_CHG_THR_VALUE, ADC_RAPID_CHANGE_THRESHOLD

/* Add threshold constants */
.set CHARGE_STOP_THRESHOLD, 80  /* ~80mV drop for charge stop detection */

/* Global variables */
.bss
.global curr_wake_status
curr_wake_status:
    .long 0

.global last_wake_status
last_wake_status:
    .long 0

.global low_threshold
low_threshold:
    .long 0

.global adaptive_threshold
adaptive_threshold:
    .long 0

#if CONFIG_ULP_BUTTON_ENABLED
.global button_press_counter
button_press_counter:
    .long 0

.global button_last_result
button_last_result:
    .long 0
#endif

.global last_result
last_result:
    .long 0

.global cum_change
cum_change:
    .long 0

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.global history_idx
history_idx:
    .long 0

.global running_sum
running_sum:
    .long 0

.global detection_phase
detection_phase:
    .long 0

.global detection_start_idx
detection_start_idx:
    .long 0

.global baseline_avg
baseline_avg:
    .long 0

.global confirmation_sum
confirmation_sum:
    .long 0

.global confirmation_count
confirmation_count:
    .long 0

.global confirmation_avg
confirmation_avg:
    .long 0

.global detection_direction
detection_direction:
    .long 0

.global charging_active
charging_active:
    .long 0

.global snapshot_confirmation_avg
snapshot_confirmation_avg:
    .long 0

.global snapshot_baseline_avg
snapshot_baseline_avg:
    .long 0

/* History snapshot variables for main CPU access */
.global snapshot_running_sum
snapshot_running_sum:
    .long 0

.global snapshot_history_idx
snapshot_history_idx:
    .long 0

.global snapshot_cycle_count
snapshot_cycle_count:
    .long 0

.global snapshot_valid_count
snapshot_valid_count:
    .long 0

.global snapshot_history_avg
snapshot_history_avg:
    .long 0

.global snapshot_last_sample
snapshot_last_sample:
    .long 0

.global snapshot_cum_change
snapshot_cum_change:
    .long 0

.global snapshot_mad
snapshot_mad:
    .long 0

.global snapshot_state
snapshot_state:
    .long 0

.global snapshot_valid
snapshot_valid:
    .long 0
#endif

/* History buffer */
.global history
history:
    .skip ADC_HISTORY_SIZE * 4  /* 4 bytes per entry */

.global cycle_count
cycle_count:
    .long 0  /* Total cycles run */

.global main_cpu_running
main_cpu_running:
    .long 0  /* Flag: 1 if main CPU is running */

.text
.global entry

/* Unified snapshot routine - expects state value in r0 */
take_snapshot_routine:
    /* Set snapshot valid flag */
    move r1, snapshot_valid
    move r2, 1
    st r2, r1, 0

    /* Snapshot running_sum */
    move r1, running_sum
    ld r2, r1, 0
    move r1, snapshot_running_sum
    st r2, r1, 0

    /* Snapshot history_idx */
    move r1, history_idx
    ld r2, r1, 0
    move r1, snapshot_history_idx
    st r2, r1, 0

    /* Snapshot cycle_count */
    move r1, cycle_count
    ld r2, r1, 0
    move r1, snapshot_cycle_count
    st r2, r1, 0

    /* Calculate and snapshot valid_count */
    move r1, cycle_count
    ld r2, r1, 0
    move r1, ADC_HISTORY_SIZE
    sub r1, r2, r1
    jump snapshot_valid_count_full_routine, ov
    move r2, ADC_HISTORY_SIZE
snapshot_valid_count_full_routine:
    move r1, snapshot_valid_count
    st r2, r1, 0

    /* Calculate and snapshot history_avg */
    move r1, running_sum
    ld r2, r1, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT
    move r1, snapshot_history_avg
    st r2, r1, 0

    /* Snapshot last_sample */
    move r1, last_result
    ld r2, r1, 0
    move r1, snapshot_last_sample
    st r2, r1, 0

    /* Snapshot cum_change */
    move r1, cum_change
    ld r2, r1, 0
    move r1, snapshot_cum_change
    st r2, r1, 0

    /* Calculate and snapshot MAD (simplified - just store 0 for now) */
    move r1, snapshot_mad
    move r2, 0
    st r2, r1, 0

    /* Snapshot state (value passed in r0) */
    and r0, r0, ULP_WAKE_CURRENT_ADC_MASK
    rsh r0, r0, ULP_WAKE_CURRENT_ADC_SHIFT
    move r1, snapshot_state
    st r0, r1, 0

    /* Return to appropriate caller based on r3 */
    move r0, 1
    sub r0, r3, r0
    jump snapshot_return_1, eq
    jump snapshot_return_2

/* Common wakeup handler for both ADC and button */
wake_main_cpu:
    /* Wake up the main CPU */
    wake  /* Wake main CPU */
    /* HALT the ULP after waking - main CPU will restart us */
    halt  /* Stop ULP execution - main CPU must restart with start_ulp_program() */

wake_skip:
    /* Main CPU already running - just continue to sleep cycle */
    jump sleep_cycle

snapshot_return_1:
    /* Reset confirmation variables after successful confirmation */
    move r1, detection_phase
    move r0, 0
    st r0, r1, 0
    
    move r1, confirmation_sum
    st r0, r1, 0
    
    move r1, confirmation_count
    st r0, r1, 0
    
    move r1, confirmation_avg
    st r0, r1, 0
    
    move r1, detection_direction
    st r0, r1, 0
    
    /* Set wake status */
    move r1, curr_wake_status
    ld r3, r1, 0
    move r1, last_wake_status
    st r3, r1, 0
    
    move r1, curr_wake_status
    st r2, r1, 0
    
    /* Only wake if main CPU is not already running */
    move r1, main_cpu_running
    ld r0, r1, 0
    move r1, 0
    sub r1, r0, r1
    jump wake_skip, eq  /* Skip wake if main CPU is running */
    jump wake_main_cpu

snapshot_return_2:
    /* Copy current to last (single load/store) */
    move r1, curr_wake_status
    ld r2, r1, 0
    move r1, last_wake_status
    st r2, r1, 0
    
    /* Set current status - preprocessor-calculated from container macro! */
    move r2, ULP_BAT_STATUS_CRITICAL_LOW         /* Auto-generated: (ADC | (LOW_THR << 2)) = 0x05 */
    move r1, curr_wake_status
    st r2, r1, 0
    
    /* Only wake if main CPU is not already running */
    move r1, main_cpu_running
    ld r0, r1, 0
    move r1, 0
    sub r1, r0, r1
    jump wake_skip, eq  /* Skip wake if main CPU is running */
    jump wake_main_cpu

.global entry
entry:
    /* Check if this is first run (cycle_count == 0) */
    move r1, cycle_count
    ld r0, r1, 0
    move r2, 0
    sub r2, r0, r2
    jump first_run_init, eq  /* If cycle_count == 0, initialize */

    /* Resume from previous state - skip initialization */
    jump wake_check

sleep_cycle:
    /* Wait for configured delay using loop (wait instruction limited to 16-bit) */
    stage_rst
    stage_inc WAIT_ITERATIONS

wait_loop:
    wait WAIT_TICKS_PER_ITER
    stage_dec 1
    jumps wait_loop, 1, ge
    
    jump main_loop

first_run_init:
#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* Initialize confirmation variables (not reset by main CPU) */
    move r1, detection_phase
    move r2, 0
    st r2, r1, 0
    
    move r1, confirmation_sum
    st r2, r1, 0
    
    move r1, confirmation_count
    st r2, r1, 0
    
    move r1, confirmation_avg
    st r2, r1, 0
    
    move r1, detection_direction
    st r2, r1, 0
#endif
    
    /* Initialize cycle_count (already reset by main CPU, but keep for safety) */
    move r1, cycle_count
    move r2, 0
    st r2, r1, 0

    /* Clear history buffer (already cleared by main CPU, but keep for safety) */
    move r1, history
    move r2, 0
    move r3, ADC_HISTORY_SIZE
clear_history_loop:
    st r2, r1, 0      /* history[i] = 0 */
    add r1, r1, 1      /* i++ */
    sub r3, r3, 1
    jump clear_history_loop, ov /* Loop if r3 > 0 */

    /* After initialization, check wake status */
    jump wake_check

wake_check:
    /* Check wake status */
    move r1, main_cpu_running
    ld r2, r1, 0
    move r0, 0
    sub r0, r2, r0  /* Check if main CPU is running */
    jump when_wake_no_process, ov  /* If main CPU is running, skip wake source check but continue processing */

    /* Main CPU is asleep - check wake status */
    move r1, curr_wake_status
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_SOURCE_MASK
    move r0, 0
    sub r0, r3, r0  /* Check if source != 0 */
    jump adc_frozen, ov  /* If wake source is set (non-zero), skip everything and sleep */

when_wake_no_process:
    /* Continue with normal processing regardless of main CPU state */
    jump main_loop

adc_frozen:
    /* ADC is frozen because wake is already set - just sleep */
    jump sleep_cycle

/* ========================================================================== */
/* BATTERY SENSING CODE - REORGANIZED FOR JUMP DISTANCE */
/* ========================================================================== */

adc_measure_and_check:
    /* Always perform ADC measurement to support confirmation phase */
    jump when_adc_start

when_adc_start:
    /* ADC measurement */
    move r0, ADC_STABILIZATION_DELAY
adc_delay:
    sub r0, r0, 1
    jump adc_delay, ov

    /* Read ADC with oversampling */
    move r0, 0
    stage_rst
adc_read_loop:
    adc r1, 0, adc_channel + 1
    add r0, r0, r1
    stage_inc 1
    jumps adc_read_loop, adc_oversampling_factor, lt

    rsh r0, r0, adc_oversampling_factor_log
    
    and r0, r0, 0xFFF
    
    /* Store ADC value */
    move r1, last_result
    st r0, r1, 0

    /* Validate ADC reading - skip if reading seems invalid (too low or too high) */
    move r1, 500   /* Minimum valid reading */
    sub r1, r0, r1
    jump sleep_cycle, ov  /* Jump if reading < 500 */
    
    move r1, 3500  /* Maximum valid reading */
    sub r1, r1, r0
    jump sleep_cycle, ov  /* Jump if reading > 3500 */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* Store current reading */
    move r1, last_result
    st r0, r1, 0
    
    /* Update history buffer */
    move r1, history_idx
    ld r2, r1, 0
    
    /* Subtract old value from running sum */
    move r1, history
    add r1, r1, r2
    ld r0, r1, 0
    
    move r1, running_sum
    ld r3, r1, 0
    sub r3, r3, r0
    st r3, r1, 0
    
    /* Add new ADC value to running sum */
    move r1, last_result
    ld r0, r1, 0
    move r1, running_sum
    ld r3, r1, 0
    add r3, r3, r0
    st r3, r1, 0
    
    /* Store new value in array */
    move r1, history_idx
    ld r2, r1, 0
    move r3, history
    add r3, r3, r2
    move r1, last_result
    ld r0, r1, 0
    st r0, r3, 0
    
    /* Advance index */
    move r1, history_idx
    ld r2, r1, 0
    add r2, r2, 1
    move r3, (ADC_HISTORY_SIZE-1)
    and r2, r2, r3
    st r2, r1, 0
    
    /* Check if buffer full */
    move r1, cycle_count
    ld r2, r1, 0
    move r3, ADC_HISTORY_SIZE
    sub r0, r2, r3
    jump sleep_cycle, ov              /* Skip if cycle_count < ADC_HISTORY_SIZE */
    
    jump adc_calc_running_avg

adc_calc_running_avg:
    /* Calculate running average: running_sum / 8 */
    /* This mimics the main processor's efficient approach */
    move r1, running_sum
    ld r2, r1, 0                   /* r2 = running_sum */
    
    /* Calculate average (divide by ULP_ADC_HISTORY_SHIFT) */
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = running_sum / 8 */

    /* === CHECK LOW BATTERY FIRST === */
    /* Skip low battery check if last wake was already low battery (anti-spam) */
    move r1, last_wake_status
    ld r3, r1, 0
    and r3, r3, ULP_WAKE_CURRENT_ADC_MASK
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    move r1, ADC_ULP_BAT_STATE_CRITICAL_LOW
    sub r0, r3, r1
    jump skip_low_battery_check, eq  /* Skip if last wake was low battery */
    
    /* Always check low threshold before rapid change detection */
    /* This prevents low battery from being masked by rapid change detection */
    move r3, low_threshold
    ld r1, r3, 0                   /* r1 = low_threshold value */
    sub r0, r2, r1                 /* r0 = avg - low_thresh */
    jump low_battery_wake, ov  /* If avg < low_thresh, immediate wake */

skip_low_battery_check:

    /* CRITICAL: Don't check rapid change on cold start (when history buffer not yet filled)
     * If cycle_count < HISTORY_SIZE, we don't have enough data for accurate comparison
     * Comparing against zeros or partial data causes false triggers! */
    move r1, cycle_count
    ld r0, r1, 0
    move r1, ULP_ADC_HISTORY_SIZE
    sub r0, r0, r1                 /* r0 = cycle_count - HISTORY_SIZE */
    jump sleep_cycle, ov              /* If cycle_count < HISTORY_SIZE, skip rapid change check */

    /* Skip rapid change check if last wake was already rapid change (anti-spam) */
    move r1, last_wake_status
    ld r3, r1, 0
    and r3, r3, ULP_WAKE_CURRENT_ADC_MASK
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    move r1, ADC_ULP_BAT_STATE_CHARGING_STARTED
    sub r0, r3, r1
    jump sleep_cycle, eq  /* Skip if last wake was charging started */
    move r1, ADC_ULP_BAT_STATE_CHARGING_STOPPED
    sub r0, r3, r1
    jump sleep_cycle, eq  /* Skip if last wake was charging stopped */

    /* Calculate absolute difference between current reading and running average */
    /* This detects RAPID changes, not cumulative drift */
    move r3, last_result
    ld r0, r3, 0              /* r0 = last_result (current reading) */
    
    /* Calculate absolute difference: |last_result - average| */
    sub r1, r0, r2            /* r1 = last_result - average */
    jump adc_positive_diff, ov /* Jump if negative (overflow) */
    jump adc_store_diff       /* Already positive, store it */
    
adc_positive_diff:
    sub r1, r2, r0            /* r1 = average - last_result (make positive) */
    
adc_store_diff:
    /* r1 now contains absolute difference */
    move r3, cum_change
    st r1, r3, 0

    /* Call enhanced detection with pre-calculated |diff| */
    jump adc_check_rapid_change_enhanced

adc_check_rapid_change_enhanced:
    /* Check if already in confirmation phase */
    move r1, detection_phase
    ld r0, r1, 0
    move r2, r0
    jump threshold_check, eq  /* If phase == 0, check for new detection */
    jump detection_already_active  /* If phase != 0, continue confirmation */

detection_already_active:
    /* Already in confirmation phase - add to confirmation sum */
    move r1, confirmation_count
    ld r0, r1, 0
    move r2, ULP_ADC_CONFIRMATION_SAMPLES
    sub r2, r0, r2
    jump confirmation_complete, eq  /* If count == CONFIRMATION_SAMPLES, check confirmation */

    /* Add current reading to confirmation sum */
    move r1, last_result
    ld r0, r1, 0              /* r0 = current reading */
    move r1, confirmation_sum
    ld r2, r1, 0              /* r2 = current sum */
    add r2, r2, r0
    st r2, r1, 0              /* confirmation_sum += current_reading */

    /* Increment confirmation count */
    move r1, confirmation_count
    ld r0, r1, 0
    add r0, r0, 1
    st r0, r1, 0

    jump main_loop

main_loop:
#if CONFIG_ULP_BUTTON_ENABLED
button_check:
    /* Read button state (0 = pressed with pull-up, 1 = released) */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + BUTTON_RTC_IO, 1)
    
    /* Store raw button state for debugging */
    move r2, button_last_result    /* r2 = address of button_last_result */
    st r1, r2, 0                   /* Store GPIO state (r1 from READ_RTC_REG) */

    /* Check if button is pressed (GPIO = 0 with pull-up) */
    jump button_pressed, eq       /* If r0 == 0, button is pressed */

    /* === BUTTON RELEASED === */
    /* Check if we had a button press in progress (counter > 0) */
    move r1, button_press_counter
    ld r2, r1, 0
    move r3, 0
    sub r3, r2, r3  /* Check if counter > 0 */
    jump button_released_after_press, ov /* If counter > 0, check threshold */
    
    /* Counter is zero - button was never pressed, continue to ADC */
    jump check_adc_timing

button_released_after_press:
    
    move r3, LONG_PRESS_THRESHOLD
    sub r3, r2, r3  /* Compare counter to threshold */
    jump button_long_press_detected, ov  /* If counter >= threshold, trigger wake */
    
    /* Threshold not met - reset counter and resume ADC */
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
    jump check_adc_timing

button_pressed:
    /* === BUTTON PRESSED === */
    /* CRITICAL: Skip ADC while button is pressed to avoid voltage noise */
    
    /* Increment button press counter */
    move r1, button_press_counter
    ld r2, r1, 0
    add r2, r2, 1
    st r2, r1, 0

    /* Don't check threshold here - avoid false triggers */
    /* Just skip ADC and go to sleep */
    jump sleep_cycle

check_adc_timing:
#endif

    /* === ADC CHECK (EVERY CYCLE for fast sampling) === */
    /* For 150kHz ULP clock, run ADC measurement on every cycle for maximum throughput */
    move r1, cycle_count
    ld r2, r1, 0
    
    /* Increment cycle count for tracking */
    add r2, r2, 1
    st r2, r1, 0
    
    /* Run ADC on EVERY cycle (no modulo check) */
    jump adc_measure_and_check

    /* Not in confirmation phase - check for new detection */
    /* Calculate adaptive threshold based on battery behavior */
    move r3, adaptive_threshold
    ld r3, r3, 0              /* r3 = current adaptive threshold */

    /* If no adaptive threshold set yet, use default */
    move r2, 0
    sub r2, r3, r2
    jump use_adaptive_threshold, ov

    /* Initialize adaptive threshold based on current voltage level */
    move r3, last_result
    ld r0, r3, 0              /* r0 = current reading */

    /* Base threshold: higher for higher voltages, lower for lower voltages */
    move r3, 50               /* Base threshold for low voltage */
    move r2, 2000
    sub r2, r0, r2
    jump set_base_threshold, ov

    move r3, 80               /* Base threshold for mid voltage */
    move r2, 2400
    sub r2, r0, r2
    jump set_base_threshold, ov

    move r3, 120              /* Base threshold for high voltage */

set_base_threshold:
    move r2, adaptive_threshold
    st r3, r2, 0              /* Store initial adaptive threshold */

use_adaptive_threshold:
    /* Use pre-calculated |diff| from cum_change */
    move r1, cum_change
    ld r1, r1, 0              /* r1 = |diff| */

    /* Apply adaptive threshold with voltage-based sensitivity */
    move r3, last_result
    ld r0, r3, 0              /* r0 = current reading */

    /* Bucket 1: Very low battery - more sensitive */
    move r3, 2000
    sub r3, r0, r3
    jump threshold_very_sensitive, ov

    /* Bucket 2: High battery - less sensitive */
    move r3, 2400
    sub r3, r0, r3
    jump threshold_less_sensitive, ov

    /* Bucket 3: Normal range - use adaptive threshold */
    move r3, adaptive_threshold
    ld r3, r3, 0
    jump threshold_check

threshold_very_sensitive:
    move r3, adaptive_threshold
    ld r3, r3, 0
    rsh r3, r3, 1             /* Half the adaptive threshold */
    jump threshold_check

threshold_less_sensitive:
    move r3, adaptive_threshold
    ld r3, r3, 0
    lsh r3, r3, 1             /* Double the adaptive threshold */
    jump threshold_check

threshold_check:
    /* r3 now contains adaptive threshold */
    /* r1 already contains |diff| from cum_change */
    sub r2, r1, r3                   /* r2 = |diff| - threshold */
    jump no_detection, ov  /* if |diff| < threshold, not detect - FIX: jump to return point */
    
no_detection:
    /* Gradually reduce adaptive threshold during normal operation */
    /* This makes detection more sensitive when battery is stable */
    move r3, adaptive_threshold
    ld r0, r3, 0              /* r0 = current adaptive threshold */

    /* Only adjust if threshold > minimum */
    move r2, 25               /* Minimum threshold */
    sub r2, r0, r2
    jump no_threshold_adjust, ov

    /* Reduce threshold by 1 (gradual adjustment) */
    sub r0, r0, 1
    st r0, r3, 0

no_threshold_adjust:
    /* Reload average for voltage level checks */
    move r3, running_sum
    ld r2, r3, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = current average */

    /* Check for charging state change based on voltage level */
    /* This detects gradual transitions that rapid change detection misses */
    move r1, charging_active
    ld r0, r1, 0
    move r1, 0
    sub r1, r0, r1
    jump check_charging_stopped, ov  /* If charging_active == 0, check if should start charging */

check_charging_started:
    /* Check if voltage > threshold for charging */
    move r0, 2200
    sub r0, r2, r0
    jump sleep_cycle, ov  /* If average <= 2200, no charging start */
    
    /* Set charging active flag */
    move r1, charging_active
    move r0, 1
    st r0, r1, 0
    
    /* Trigger charging started */
    move r2, ULP_BAT_STATUS_CHARGING_STARTED
    jump set_charge_wake_status

check_charging_stopped:
    /* Check if voltage < threshold for charging stopped */
    move r0, 2000
    sub r0, r2, r0
    jump sleep_cycle, ov  /* If average >= 2000, still charging */
    
    /* Clear charging active flag */
    move r1, charging_active
    move r0, 0
    st r0, r1, 0
    
    /* Trigger charging stopped */
    move r2, ULP_BAT_STATUS_CHARGING_STOPPED
    jump set_charge_wake_status

adc_check_thresholds:
    /* Check running average < ADC_LOW_THR_VALUE (use avg for stability, not noisy last_result) */
    move r3, running_sum
    ld r0, r3, 0
    rsh r0, r0, ULP_ADC_HISTORY_SHIFT  /* r0 = avg */
    move r1, low_threshold   /* Direct immediate value, no RAM needed! */
    ld r1, r1, 0
    sub r0, r0, r1
    jump low_battery_wake, ov

    jump sleep_cycle

    /* === DETECTION TRIGGERED - Start confirmation phase === */
    jump start_new_detection

start_new_detection:
    /* Initialize confirmation phase */
    move r1, detection_phase
    move r0, 1
    st r0, r1, 0                 /* detection_phase = 1 */

    /* Store initial reading for confirmation */
    move r1, confirmation_sum
    move r0, 0
    st r0, r1, 0                 /* confirmation_sum = 0 */

    /* Store current reading as first confirmation sample */
    move r1, last_result
    ld r0, r1, 0
    move r1, confirmation_sum
    st r0, r1, 0                 /* confirmation_sum = current_reading */

    /* Set confirmation count to 1 */
    move r1, confirmation_count
    move r0, 1
    st r0, r1, 0

    /* Store direction (positive = rise, negative = fall) */
    move r3, last_result
    ld r0, r3, 0              /* r0 = current reading */
    move r3, running_sum
    ld r2, r3, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = running average */
    sub r1, r0, r2            /* r1 = current - average */
    move r3, detection_direction
    st r1, r3, 0              /* Store direction */

    jump main_loop

confirmation_complete:
    /* Calculate average of confirmation samples */
    move r1, confirmation_sum
    ld r0, r1, 0              /* r0 = sum */
    
    /* Approximate division by 3: average ≈ (sum >> 2) + (sum >> 4) + (sum >> 6) */
    move r2, r0
    rsh r2, r2, 2             /* r2 = sum / 4 */
    move r3, r0
    rsh r3, r3, 4             /* r3 = sum / 16 */
    add r2, r2, r3            /* r2 = sum/4 + sum/16 */
    move r3, r0
    rsh r3, r3, 6             /* r3 = sum / 64 */
    add r2, r2, r3            /* r2 ≈ sum / 3 */
    
    move r1, confirmation_avg
    st r2, r1, 0              /* Store confirmation average */

    /* Compare with running average to confirm direction */
    move r3, running_sum
    ld r2, r3, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = running average */
    sub r1, r0, r2            /* r1 = confirmation_avg - running_avg */

    /* Check if direction matches initial detection */
    move r3, detection_direction
    ld r0, r3, 0              /* r0 = initial direction */
    /* Check if both have same sign: (r0 >= 0 && r1 >= 0) || (r0 < 0 && r1 < 0) */
    move r3, r0
    rsh r3, r3, 15            /* r3 = sign bit of r0 (1 if negative, 0 if positive) */
    move r2, r1
    rsh r2, r2, 15            /* r2 = sign bit of r1 (1 if negative, 0 if positive) */
    sub r3, r3, r2            /* r3 = 0 if signs match, non-zero if different */
    jump confirmation_passed, eq  /* If signs match, confirmation passed */

    /* Direction changed - reset detection */
    jump confirmation_failed

set_charge_wake_status:
    /* For CHARGING_STOPPED, check if charging is actually active */
    move r0, ULP_BAT_STATUS_CHARGING_STOPPED
    sub r0, r2, r0
    jump check_charging_active, eq
    
    /* For CHARGING_STARTED, charging_active is already set in confirmation_passed */
    jump anti_spam_check

check_charging_active:
    /* Check if charging is currently active */
    move r1, charging_active
    ld r0, r1, 0
    move r1, 0
    sub r1, r0, r1
    jump sleep_cycle, eq              /* Skip if charging not active */
    
    /* Clear charging active flag */
    move r1, charging_active
    move r0, 0
    st r0, r1, 0
    
anti_spam_check:
    /* Check if main CPU is running - if so, allow status updates without anti-spam */
    move r1, main_cpu_running
    ld r0, r1, 0
    move r1, 0
    sub r1, r0, r1
    jump take_snapshot, eq  /* Skip anti-spam if main CPU is running */
    
    /* Anti-spam check - only when main CPU is not running */
    move r1, last_wake_status
    ld r3, r1, 0
    and r3, r3, ULP_WAKE_CURRENT_ADC_MASK
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    
    sub r0, r2, r3
    jump sleep_cycle, eq              /* Skip if same reason */
    
take_snapshot:
    /* Take snapshot of current state before waking/updating */
    /* Pass wake reason in r0 and return label index in r3 */
    move r0, r2
    move r3, 1  /* Return to snapshot_return_1 */
    jump take_snapshot_routine

confirmation_passed:
    /* Set snapshot variables for main CPU decision confirmations */
    move r3, confirmation_avg
    ld r0, r3, 0              /* r0 = confirmed average */
    move r3, snapshot_confirmation_avg
    st r0, r3, 0              /* snapshot_confirmation_avg = confirmation_avg */

    move r3, running_sum
    ld r0, r3, 0
    rsh r0, r0, ULP_ADC_HISTORY_SHIFT  /* r0 = running average (baseline) */
    move r3, snapshot_baseline_avg
    st r0, r3, 0              /* snapshot_baseline_avg = running_avg */

    /* Update adaptive threshold based on successful detection */
    move r3, cum_change
    ld r0, r3, 0              /* r0 = magnitude of change that triggered detection */

    /* Calculate new adaptive threshold: current_threshold + (change_magnitude / 4) */
    /* This makes threshold more sensitive to similar changes */
    move r3, adaptive_threshold
    ld r2, r3, 0              /* r2 = current adaptive threshold */
    rsh r0, r0, 2             /* r0 = change_magnitude / 4 */
    add r2, r2, r0            /* r2 = new adaptive threshold */

    /* Prevent threshold from becoming too sensitive (min 20) or insensitive (max 200) */
    move r0, 20
    sub r0, r2, r0
    jump threshold_too_low, ov
    move r0, 200
    sub r0, r2, r0
    jump threshold_too_high, ov
    jump store_new_threshold

threshold_too_low:
    move r2, 20
    jump store_new_threshold

threshold_too_high:
    move r2, 200

store_new_threshold:
    move r3, adaptive_threshold
    st r2, r3, 0              /* Store updated adaptive threshold */

    /* Direction confirmed - determine wake reason */
    move r3, detection_direction
    ld r1, r3, 0              /* r1 = direction */

    jump confirmation_fall_detected, ov  /* If negative, fall detected */

    /* Rise detected - set charging active flag */
    move r1, charging_active
    move r0, 1
    st r0, r1, 0              /* charging_active = 1 */
    move r2, ULP_BAT_STATUS_CHARGING_STARTED
    jump set_charge_wake_status

confirmation_fall_detected:
    /* Check if this is low battery or charging stopped */
    move r3, running_sum
    ld r2, r3, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT
    move r0, 2100
    sub r0, r2, r0
    jump low_battery_wake, ov

    /* Rapid change fall - CHARGING_STOPPED */
    move r2, ULP_BAT_STATUS_CHARGING_STOPPED
    jump set_charge_wake_status

confirmation_failed:
    /* Reset all detection and confirmation variables */
    move r1, detection_phase
    move r0, 0
    st r0, r1, 0
    
    move r1, confirmation_sum
    st r0, r1, 0
    
    move r1, confirmation_count
    st r0, r1, 0
    
    move r1, confirmation_avg
    st r0, r1, 0
    
    move r1, detection_direction
    st r0, r1, 0
    
    jump main_loop
low_battery_wake:
    /* Check if same as last wake reason (anti-spam) - OPTIMIZED! */
    move r1, last_wake_status
    ld r2, r1, 0
    and r2, r2, ULP_WAKE_CURRENT_ADC_MASK
    rsh r2, r2, ULP_WAKE_CURRENT_ADC_SHIFT
    
    /* If last reason is NONE (0), always wake - this is first wake */
    jump adc_low_set_wake, eq
    
    /* Get current ADC reason (we know it's LOW_THR=1) */
    move r3, ULP_BAT_STATUS_CRITICAL_LOW
    
    /* Compare with last */
    sub r0, r3, r2
    jump sleep_cycle, eq       /* Skip only if same non-zero reason */
    
adc_low_set_wake:
    /* Take snapshot of current state before waking */
    /* Pass wake reason in r0 and return label index in r3 */
    move r0, ULP_BAT_STATUS_CRITICAL_LOW
    move r3, 2  /* Return to snapshot_return_2 */
    jump take_snapshot_routine
#endif

#if CONFIG_ULP_BUTTON_ENABLED
button_long_press_detected:
    /* Set button wake - OPTIMIZED: copy current->last in 2 instructions! */
    
    /* Copy current to last (single load/store) */
    move r1, curr_wake_status
    ld r2, r1, 0
    move r1, last_wake_status
    st r2, r1, 0
    
    /* Set new current status - preprocessor-calculated from container macro! */
    move r2, ULP_WAKE_STATUS_BUTTON_LONG_PRESS   /* Auto-generated: (BUTTON | (LONG_PRESS << 5)) = 0x22 */
    move r1, curr_wake_status
    st r2, r1, 0
    
    /* Reset button counter */
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
    jump wake_main_cpu
#endif