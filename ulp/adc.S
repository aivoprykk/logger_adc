/*
 adc.S
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "../ulp_adc_config.h"

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

/* Global variables */
.bss
/* ULP memory is 32-bit word addressed - must use .long for reliable access */
/* For values that fit in 16 bits, we use only lower 16 bits of the 32-bit word */

/* Wake status - 2 variables instead of 1 (saves code) or 6 (saves data) */
.global curr_wake_status
curr_wake_status:
    .long 0  /* Packed: bits 0-1=source, 2-4=adc_reason, 5-7=button_reason */
.global last_wake_status
last_wake_status:
    .long 0  /* Packed: bits 0-1=source, 2-4=adc_reason, 5-7=button_reason */
.global low_threshold
low_threshold:
    .long 0
#if CONFIG_ULP_BUTTON_ENABLED
.global button_press_counter
button_press_counter:
    .long 0  /* ULP st instruction writes 32-bit, must use .long */
.global button_last_result
button_last_result:
    .long 0  /* ULP st instruction writes 32-bit, must use .long */
#endif

.global last_result
last_result:
    .long 0  /* 32-bit word, only lower 12 bits used (ADC reading max 4095) */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.global cum_change
cum_change:
    .long 0  /* 32-bit word, only lower 16 bits used */
.global history
history: 
    .space (ADC_HISTORY_SIZE * 4)  /* 4 elements * 4 bytes, only lower 12 bits used */
.global history_idx
history_idx:
    .long 0  /* 32-bit word, only lower 16 bits used (index 0-3) */
.global running_sum
running_sum:
    .long 0  /* 32-bit: full word used (sum can be 4*4095=16380) */

/* Snapshot variables: written by ULP immediately before waking the main CPU
 * so the CPU can later inspect the exact ULP state that caused the wake.
 * Only lower 16 bits are used for most fields, snapshot_valid is 0/1. */
.global snapshot_valid
snapshot_valid:
    .long 0
.global snapshot_running_sum
snapshot_running_sum:
    .long 0
.global snapshot_history_idx
snapshot_history_idx:
    .long 0
.global snapshot_cycle_count
snapshot_cycle_count:
    .long 0
.global snapshot_last_result
snapshot_last_result:
    .long 0
#if CONFIG_ULP_MAD_ENABLED
.global snapshot_mad
snapshot_mad:
    .long 0
.global snapshot_state
snapshot_state:
    .long 0
#endif
.global detection_phase
detection_phase:
    .long 0  /* 0=idle, 1=waiting confirmation */

.global detection_start_idx  
detection_start_idx:
    .long 0  /* history_idx when detection started */

.global baseline_avg
baseline_avg:
    .long 0  /* running_avg at detection time */

.global detection_direction
detection_direction:
    .long 0  /* 0=fall, 1=rise */

.global snapshot_detection_start_idx
snapshot_detection_start_idx:
    .long 0

.global snapshot_baseline_avg
snapshot_baseline_avg:
    .long 0

.global snapshot_detection_direction  
snapshot_detection_direction:
    .long 0

/* Add threshold constants */
.set CHARGE_STOP_THRESHOLD, 80  /* ~80mV drop for charge stop detection */
#endif

.global cycle_count
cycle_count:
    .long 0  /* 32-bit: full word used (can grow large) */

.text
.global entry
entry:
    /* CRITICAL: Check wake status FIRST, before anything else!
     * If a wake event is pending, we MUST skip ALL processing to preserve the wake reason.
     * This prevents ADC from overwriting button wake status. */
    move r1, curr_wake_status
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_SOURCE_MASK
    move r0, 0
    sub r0, r3, r0  /* Check if source != 0 */
    jump adc_frozen, ov  /* If wake source is set (non-zero), skip everything and sleep */

    /* DON'T clear wake_data here - main CPU clears it via adc_ulp_clear_wake_sources() */
    /* This preserves wake_data so main CPU can read it after ULP wake */

    /* DON'T reset cycle_count here - let it continue across restarts */
    /* This prevents timing issues when ULP is restarted frequently */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* Initialize first time flag check for ADC */
    /* First run if both history_idx==0 AND cycle_count==0 */
    move r1, history_idx
    ld r2, r1, 0
    jump check_cycle_count, eq  // If idx is 0, check cycle_count
    jump main_loop              // idx != 0, not first run
    
check_cycle_count:
    move r1, cycle_count
    ld r2, r1, 0
    jump first_run_init, eq     // If cycle_count is also 0, do initialization
#endif
    jump main_loop

first_run_init:
#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* First run - initialize ALL variables */
    move r1, history_idx
    move r2, 0
    st r2, r1, 0
    
    move r1, running_sum
    move r2, 0
    st r2, r1, 0
#endif
    
    /* Initialize cycle_count only on first run */
    move r1, cycle_count
    move r2, 0
    st r2, r1, 0

#if CONFIG_ULP_BUTTON_ENABLED
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
#endif

    /* Clear wake status variables */
    move r2, 0
    move r1, curr_wake_status
    st r2, r1, 0
    move r1, last_wake_status
    st r2, r1, 0

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

    /* Don't check threshold here - wait for release to avoid false triggers */
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

/* ========================================================================== */
/* BATTERY SENSING CODE - EXACTLY FROM YOUR TESTED battery_sensing.S */
/* ========================================================================== */

adc_measure_and_check:
    /* Check if we already have a wake event pending - if so, FREEZE all ADC processing */
    move r1, curr_wake_status
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_SOURCE_MASK
    move r0, 0
    sub r0, r3, r0  /* Check if source != 0 */
    jump adc_frozen, ov  /* If wake source is set (non-zero), skip all ADC processing */

    /* === ADC MEASUREMENT === */
    /* Delay to stabilize ADC */
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
    
    /* Now store clean ADC value */
    move r1, last_result
    st r0, r1, 0

    /* Validate ADC reading - skip if reading seems invalid (too low or too high) */
    move r1, 500   /* Minimum valid reading */
    sub r1, r0, r1
    jump adc_exit, ov  /* Jump if reading < 500 */
    
    move r1, 3500  /* Maximum valid reading */
    sub r1, r1, r0
    jump adc_exit, ov  /* Jump if reading > 3500 */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* Store current reading */
    move r1, last_result
    st r0, r1, 0
    
    /* 1. Get current index value */
    move r1, history_idx       /* r1 = address of index var */
    ld r2, r1, 0                   /* r2 = index value (0-3) */
    
    /* 2. ALWAYS subtract old value from running sum (even if buffer not full, we'll add it back) */
    /* Get old value at current index */
    move r1, history           /* r1 = array base */
    add r1, r1, r2                 /* r1 = &history[index] */
    ld r0, r1, 0                   /* r0 = old value at this index */
    
    /* Subtract from running sum */
    move r1, running_sum
    ld r3, r1, 0                   /* r3 = current running_sum */
    sub r3, r3, r0                 /* running_sum -= old_value */
    st r3, r1, 0                   /* store updated sum */
    /* 3. Add new ADC value to running sum */
    move r1, last_result
    ld r0, r1, 0                   /* r0 = new ADC value */
    move r1, running_sum
    ld r3, r1, 0                   /* r3 = current running_sum */
    add r3, r3, r0                 /* running_sum += new_value */
    st r3, r1, 0                   /* store updated sum */
    
    /* 4. Store new ADC value in array at current index */
    move r1, history_idx
    ld r2, r1, 0                   /* r2 = current index */
    move r3, history           /* r3 = array base */
    lsh r1, r2, 0                  /* r1 = index * 1 (word addressing) */
    add r3, r3, r1                 /* r3 = &history[index] */
    move r1, last_result
    ld r0, r1, 0                   /* r0 = new ADC value */
    st r0, r3, 0                   /* Store in array */
    
    /* 5. Advance index: (index + 1) % ADC_HISTORY_SIZE */
    move r1, history_idx
    ld r2, r1, 0                   /* r2 = current index */
    add r2, r2, 1                  /* index + 1 */
    move r3, (ADC_HISTORY_SIZE-1) /* FIX: 7 for size 8 */
    and r2, r2, r3                 /* wrap 0-7 */
    st r2, r1, 0                   /* store new index */
    
    /* 6. Only process if we have at least ADC_HISTORY_SIZE samples (cycle_count >= 4) */
    move r1, cycle_count
    ld r2, r1, 0
    move r3, ADC_HISTORY_SIZE
    sub r0, r2, r3
    jump adc_exit, ov              /* Skip if cycle_count < ADC_HISTORY_SIZE */
    
    jump adc_calc_running_avg

/* Simple voltage-based adaptive thresholds */
adc_get_simple_adaptive_threshold:
    move r1, last_result
    ld r0, r1, 0
    
    /* Bucket 1: Very low battery < ~3.6V (raw < 2000) - most sensitive */
    move r1, 2000  
    sub r1, r0, r1
    jump very_sensitive, ov
    
    /* Bucket 2: High battery > ~4.1V (raw > 2400) - least sensitive */  
    move r1, 2400
    sub r1, r0, r1
    jump less_sensitive, ov
    
    /* Bucket 3: Normal range (default sensitivity) */
    move r0, ADC_RAPID_CHG_THR_VALUE
    jump threshold_done
    
very_sensitive:
    move r0, ADC_RAPID_CHG_THR_VALUE
    rsh r0, r0, 1  /* Divide by 2 */
    add r0, r0, ADC_RAPID_CHG_THR_VALUE
    rsh r0, r0, 2  /* Approximately ×0.75 */
    jump threshold_done
    
less_sensitive:
    move r0, ADC_RAPID_CHG_THR_VALUE
    lsh r0, r0, 1  /* Multiply by 2 */
    add r0, r0, ADC_RAPID_CHG_THR_VALUE
    rsh r0, r0, 2  /* Approximately ×1.25 */
    
threshold_done:
    /* r0 now contains adaptive threshold */
    /* In ULP, we can't return properly, so caller must handle this */
    /* We'll use r3 to pass the threshold back */
    move r3, r0
    
    /* Since we can't return, we need to jump back to a known point */
    /* This is a limitation of ULP - we have to inline or use fixed return points */
    jump adc_threshold_return

/* Enhanced detection with inline threshold calculation */
adc_check_rapid_change_enhanced:
    /* Get running average (baseline) */
    move r1, running_sum
    ld r2, r1, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* baseline_avg */
    
    /* Get current reading */
    move r1, last_result
    ld r0, r1, 0
    
    /* Calculate difference from baseline */
    sub r1, r0, r2            /* diff = current - baseline_avg */
    
    /* === INLINE THRESHOLD CALCULATION === */
    move r3, last_result
    ld r0, r3, 0
    
    /* Bucket 1: Very low battery */
    move r3, 2000  
    sub r3, r0, r3
    jump threshold_very_sensitive, ov
    
    /* Bucket 2: High battery */  
    move r3, 2400
    sub r3, r0, r3
    jump threshold_less_sensitive, ov
    
    /* Bucket 3: Normal range */
    move r3, ADC_RAPID_CHG_THR_VALUE
    jump threshold_check
    
threshold_very_sensitive:
    move r3, ADC_RAPID_CHG_THR_VALUE
    rsh r3, r3, 1
    add r3, r3, ADC_RAPID_CHG_THR_VALUE
    rsh r3, r3, 2  /* ×0.75 */
    jump threshold_check
    
threshold_less_sensitive:
    move r3, ADC_RAPID_CHG_THR_VALUE
    lsh r3, r3, 1
    add r3, r3, ADC_RAPID_CHG_THR_VALUE
    rsh r3, r3, 2  /* ×1.25 */
    
threshold_check:
    /* r3 now contains adaptive threshold */
    /* Continue with detection logic */
    sub r1, r1, r3
    jump potential_charge_event, ov  /* If diff >= threshold */
    
    jump check_confirmation_phase

adc_threshold_return:
    /* This is where the threshold function would return to */
    /* But since we inlined it, we don't need this anymore */
    jump adc_exit

potential_charge_event:
    /* === PHASE 1: Detection - save state and take snapshot === */
    move r1, detection_phase
    move r2, 1
    st r2, r1, 0                  /* Set phase = 1 (waiting confirmation) */
    
    move r1, detection_start_idx
    move r2, history_idx
    ld r2, r2, 0
    st r2, r1, 0                  /* Save starting index */
    
    move r1, baseline_avg
    move r2, running_sum
    ld r2, r2, 0
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT
    st r2, r1, 0                  /* Save baseline average */
    
    /* Determine direction */
    move r1, detection_direction
    move r2, 0
    sub r2, r0, r2                /* Check if current > 0 (always true) */
    jump set_rise_direction, ov
set_rise_direction:
    move r2, 1                    /* 1 = rise */
    st r2, r1, 0                  /* Save direction */
    
    /* Take snapshot at detection moment */
    jump write_detection_snapshot
    
    jump adc_exit

/* Check if we're in confirmation phase */
check_confirmation_phase:
    move r1, detection_phase
    ld r0, r1, 0
    move r1, 1
    sub r1, r0, r1
    jump do_confirmation_check, eq  /* If phase == 1 */
    jump adc_exit

do_confirmation_check:
    /* === PHASE 2: Confirmation (2-3 readings later) === */
    move r1, detection_start_idx
    ld r2, r1, 0
    
    /* Check if we have 2 new readings since detection */
    move r1, history_idx
    ld r0, r1, 0
    sub r1, r0, r2                /* cycles_since_detection */
    and r1, r1, (ADC_HISTORY_SIZE-1)
    
    move r0, 2                    /* Require 2 new readings (400ms) */
    sub r1, r1, r0
    jump adc_exit, ov             /* Not enough readings yet */
    
    /* Calculate average of 2 confirmation readings */
    move r0, 0                    /* sum = 0 */
    move r1, 2                    /* count = 2 */
    
    /* Start from detection index + 1 */
    move r2, detection_start_idx
    ld r2, r2, 0
    add r2, r2, 1
    and r2, r2, (ADC_HISTORY_SIZE-1)
    
confirmation_sum_loop:
    move r3, history
    add r3, r3, r2
    ld r3, r3, 0                  /* reading at index */
    add r0, r0, r3                /* sum += reading */
    
    /* Next index */
    add r2, r2, 1
    and r2, r2, (ADC_HISTORY_SIZE-1)
    
    sub r1, r1, 1
    jump confirmation_sum_loop, ov /* Loop 2 times */
    
    /* Calculate confirmation average: sum / 2 */
    rsh r0, r0, 1                 /* Efficient divide by 2 */
    
    /* Compare with baseline */
    move r1, baseline_avg
    ld r3, r1, 0
    sub r1, r0, r3                /* confirmation_avg - baseline_avg */
    
    /* Get threshold */
    jump adc_get_simple_adaptive_threshold
    /* r3 = threshold */
    
    /* Check direction and threshold */
    move r2, detection_direction
    ld r2, r2, 0
    move r0, 0
    sub r0, r2, r0
    jump check_rise_confirmation, eq  /* If direction = rise */
    
check_fall_confirmation:
    /* For fall: confirmation_avg should be below baseline - threshold */
    add r1, r1, r3                /* diff + threshold */
    jump confirmation_failed, ov   /* If diff > -threshold, failed */
    jump confirmation_passed
    
check_rise_confirmation:  
    /* For rise: confirmation_avg should be above baseline + threshold */
    sub r1, r1, r3
    jump confirmation_failed, ov   /* If diff < threshold, failed */
    /* fall through to confirmation_passed */

confirmation_passed:
    /* === CONFIRMED! Trigger wake === */
    move r1, detection_phase
    move r2, 0
    st r2, r1, 0                  /* Reset phase */
    
    /* Set wake reason based on direction */
    move r2, detection_direction
    ld r2, r2, 0
    move r0, 0
    sub r0, r2, r0
    jump trigger_charge_start, eq
    
trigger_charge_stop:
    move r2, ULP_WAKE_STATUS_ADC_RAPID_FALL
    jump set_charge_wake_status

trigger_charge_start:
    move r2, ULP_WAKE_STATUS_ADC_RAPID_RISE
    /* fall through */

set_charge_wake_status:
    /* Anti-spam check */
    move r1, last_wake_status
    ld r3, r1, 0
    and r3, r3, ULP_WAKE_CURRENT_ADC_MASK
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    
    sub r0, r2, r3
    jump adc_exit, eq              /* Skip if same reason */
    
    /* Set wake status */
    move r1, curr_wake_status
    ld r3, r1, 0
    move r1, last_wake_status
    st r3, r1, 0
    
    move r1, curr_wake_status
    st r2, r1, 0
    jump write_snapshot_and_wake

confirmation_failed:
    /* Reset detection phase */
    move r1, detection_phase
    move r2, 0
    st r2, r1, 0
    jump adc_exit

/* Take snapshot at detection moment (not confirmation) */
write_detection_snapshot:
    /* Save standard ULP state */
    move r3, snapshot_running_sum
    move r1, running_sum
    ld r0, r1, 0
    st r0, r3, 0

    move r3, snapshot_history_idx
    move r1, history_idx
    ld r0, r1, 0
    st r0, r3, 0

    move r3, snapshot_cycle_count
    move r1, cycle_count
    ld r0, r1, 0
    st r0, r3, 0

    move r3, snapshot_last_result
    move r1, last_result
    ld r0, r1, 0
    st r0, r3, 0

    /* Save detection-specific data */
    move r3, snapshot_detection_start_idx
    move r1, detection_start_idx
    ld r0, r1, 0
    st r0, r3, 0

    move r3, snapshot_baseline_avg
    move r1, baseline_avg
    ld r0, r1, 0
    st r0, r3, 0

    move r3, snapshot_detection_direction
    move r1, detection_direction
    ld r0, r1, 0
    st r0, r3, 0

    /* Mark snapshot as valid */
    move r3, snapshot_valid
    move r0, 1
    st r0, r3, 0

    jump adc_exit  /* Return from function */

adc_calc_running_avg:
    /* Calculate running average: running_sum / 8 */
    /* This mimics the main processor's efficient approach */
    move r1, running_sum
    ld r2, r1, 0                   /* r2 = running_sum */
    
    /* Calculate average (divide by ULP_ADC_HISTORY_SHIFT) */
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = running_sum / 8 */

    /* CRITICAL: Don't check rapid change on cold start (when history buffer not yet filled)
     * If cycle_count < HISTORY_SIZE, we don't have enough data for accurate comparison
     * Comparing against zeros or partial data causes false triggers! */
    move r1, cycle_count
    ld r0, r1, 0
    move r1, ULP_ADC_HISTORY_SIZE
    sub r0, r0, r1                 /* r0 = cycle_count - HISTORY_SIZE */
    jump adc_check_thresholds, ov  /* If cycle_count < HISTORY_SIZE, skip rapid change check */

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

    /* OLD: Direct threshold check */
    /* move r3, cum_change
    ld r0, r3, 0
    move r1, ADC_RAPID_CHG_THR_VALUE
    sub r0, r0, r1
    jump adc_rapid_change_triggered, eq
    jump adc_rapid_change_triggered, ov
    jump adc_check_thresholds */

    /* NEW: Enhanced detection */
    jump adc_check_rapid_change_enhanced
    
    /* Continue to low threshold check */
    jump adc_check_thresholds

#endif

adc_check_thresholds:
    /* Check last_result < ADC_LOW_THR_VALUE (compile-time constant) */
    move r3, last_result
    ld r0, r3, 0
    move r1, low_threshold   /* Direct immediate value, no RAM needed! */
    ld r1, r1, 0
    sub r0, r0, r1
    jump adc_low_threshold_triggered, ov

    jump adc_exit

adc_frozen:
    /* ADC is frozen because wake is already set - just sleep */
    jump sleep_cycle

adc_exit:
    jump sleep_cycle

adc_low_threshold_triggered:
    /* Check if same as last wake reason (anti-spam) - OPTIMIZED! */
    move r1, last_wake_status
    ld r2, r1, 0
    and r2, r2, ULP_WAKE_CURRENT_ADC_MASK
    rsh r2, r2, ULP_WAKE_CURRENT_ADC_SHIFT
    
    /* If last reason is NONE (0), always wake - this is first wake */
    jump adc_low_set_wake, eq
    
    /* Get current ADC reason (we know it's LOW_THR=1) */
    move r3, ULP_ADC_WAKE_LOW_THR
    
    /* Compare with last */
    sub r0, r3, r2
    jump adc_exit, eq       /* Skip only if same non-zero reason */
    
adc_low_set_wake:
    /* Copy current to last (single load/store) */
    move r1, curr_wake_status
    ld r2, r1, 0
    move r1, last_wake_status
    st r2, r1, 0
    
    /* Set current status - preprocessor-calculated from container macro! */
    move r2, ULP_WAKE_STATUS_ADC_LOW_THR         /* Auto-generated: (ADC | (LOW_THR << 2)) = 0x05 */
    move r1, curr_wake_status
    st r2, r1, 0

#if CONFIG_ULP_MAD_ENABLED
    /* For low-battery wake we skip MAD computation to save cycles.
     * Explicitly set snapshot_mad=0 and snapshot_state to LOW_THR. */
    move r1, 0
    move r2, ULP_WAKE_STATUS_ADC_LOW_THR
#endif
    jump write_snapshot_and_wake

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
adc_rapid_change_triggered:
    /* Check if same as last wake reason (anti-spam) - OPTIMIZED! */
    move r1, last_wake_status
    ld r2, r1, 0
    and r2, r2, ULP_WAKE_CURRENT_ADC_MASK
    rsh r2, r2, ULP_WAKE_CURRENT_ADC_SHIFT
    
    /* If last reason is NONE (0), always wake - this is first wake */
    jump adc_rapid_set_wake, eq
    
    /* Get current ADC reason (we know it's RAPID_CHG=3) */
    move r3, ULP_ADC_WAKE_RAPID_CHG
    
    /* Compare with last */
    sub r0, r3, r2
    jump adc_exit, eq       /* Skip only if same non-zero reason */
    
adc_rapid_set_wake:
    /* Copy current to last (single load/store) */
    move r1, curr_wake_status
    ld r2, r1, 0
    move r1, last_wake_status
    st r2, r1, 0
    
    /* Set current status - preprocessor-calculated from container macro! */
    move r2, ULP_WAKE_STATUS_ADC_RAPID_CHG       /* Auto-generated: (ADC | (RAPID_CHG << 2)) = 0x09 */
    move r1, curr_wake_status
    st r2, r1, 0

    /* Compute MAD over history if buffer is full and set snapshot_state in r2,
     * put computed MAD in r1 then branch to common writer. */
    move r1, cycle_count
    ld r2, r1, 0
    move r3, ADC_HISTORY_SIZE
    sub r0, r2, r3
    /* MAD computation removed for ULP (register/instruction limits). Fall through
     * to the skip path which writes snapshot_mad=0 and snapshot_state. */

#if CONFIG_ULP_MAD_ENABLED
    /* Skip computing MAD on rapid-change too (ULP register and instruction limits).
     * Set snapshot_mad=0 and set state to RAPID_CHG. CPU will compute MAD if needed. */
    move r1, 0
    move r2, ULP_WAKE_STATUS_ADC_RAPID_CHG
#endif
    jump write_snapshot_and_wake

/* Common snapshot writer: expects
 *  r1 = snapshot_mad (or computed MAD),
 *  r2 = snapshot_state (ULP wake status)
 * Writes running_sum/history_idx/cycle_count/last_result, then snapshot_mad/state, then sets snapshot_valid and wakes CPU.
 */
write_snapshot_and_wake:
    /* Use only r0..r3. r1/r2 hold snapshot_mad/snapshot_state on entry. */
    /* running_sum -> snapshot_running_sum */
    move r3, running_sum
    ld r0, r3, 0
    move r3, snapshot_running_sum
    st r0, r3, 0

    /* history_idx -> snapshot_history_idx */
    move r3, history_idx
    ld r0, r3, 0
    move r3, snapshot_history_idx
    st r0, r3, 0

    /* cycle_count -> snapshot_cycle_count */
    move r3, cycle_count
    ld r0, r3, 0
    move r3, snapshot_cycle_count
    st r0, r3, 0

    /* last_result -> snapshot_last_result */
    move r3, last_result
    ld r0, r3, 0
    move r3, snapshot_last_result
    st r0, r3, 0

#if CONFIG_ULP_MAD_ENABLED
    /* r1 contains MAD (if computed), write it to snapshot_mad */
    move r3, snapshot_mad
    st r1, r3, 0

    /* r2 contains the snapshot state enum */
    move r3, snapshot_state
    st r2, r3, 0
#endif

    move r3, snapshot_valid
    move r0, 1
    st r0, r3, 0

    jump wake_main_cpu
#endif

/* Common wakeup handler for both ADC and button */
wake_main_cpu:
    /* Wake up the main CPU */
    wake  /* Wake main CPU */
    /* HALT the ULP after waking - main CPU will restart us */
    halt  /* Stop ULP execution - main CPU must restart with start_ulp_program() */

sleep_cycle:
    /* Never clear wake_data once set - only main CPU can clear it via entry point */
    /* Wait for configured delay using loop (wait instruction limited to 16-bit) */
    stage_rst
    stage_inc WAIT_ITERATIONS

wait_loop:
    wait WAIT_TICKS_PER_ITER
    stage_dec 1
    jumps wait_loop, 1, ge  /* Continue if stage >= 1 */
    
    jump main_loop
