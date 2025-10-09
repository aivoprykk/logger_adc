/*
 adc.S
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "../ulp_adc_config.h"

/* Configuration from preprocessor flags */
#if CONFIG_ULP_BUTTON_ENABLED
.set BUTTON_RTC_IO, CONFIG_ULP_BUTTON_RTC_IO  
.set LONG_PRESS_THRESHOLD, ULP_LONG_PRESS_CYCLES
#endif

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.set ADC_HISTORY_SIZE, ULP_ADC_HISTORY_SIZE
#endif

.set ADC_STABILIZATION_DELAY, ULP_ADC_STABILIZATION_DELAY
.set adc_channel, ULP_ADC_CHANNEL
.set adc_oversampling_factor_log, ULP_ADC_OVERSAMPLING
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

.set CYCLE_WAIT_TICKS, (8000 * ULP_CYCLE_TIME_MS / 1000)

/* Global variables */
.bss
.global wake_data
wake_data:
    .long 0  /* Single packed variable */

#if CONFIG_ULP_BUTTON_ENABLED
.global button_press_counter
button_press_counter:
    .long 0
.global button_last_result
button_last_result:
    .long 0  // For debugging: store raw button state
#endif

/* Battery sensing variables - EXACTLY from your tested code */
.global low_thr
low_thr:
    .long 0
.global last_result
last_result:
    .long 0
#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.global rapid_change_thr
rapid_change_thr:
    .long 0
.global cum_change
cum_change:
    .long 0
.global prev_result
prev_result: 
    .space (ADC_HISTORY_SIZE * 4)  // 8 elements * 4 bytes
.global prev_result_idx
prev_result_idx:
    .long 0
.global sample_count
sample_count:
    .long 0
.global running_sum
running_sum:
    .long 0
#endif
.global debug_counter
debug_counter:
    .long 0
.global cycle_count
cycle_count:
    .long 0

.text
.global entry
entry:
    /* Initialize - don't clear wake_data entirely, just current wake source */
    /* This preserves last wake reasons across ULP restarts */
    move r1, wake_data
    ld r2, r1, 0
    /* Clear only current wake source, preserve last wake reasons */
    and r2, r2, ~ULP_WAKE_CURRENT_SOURCE_MASK
    st r2, r1, 0

    /* DON'T reset cycle_count here - let it continue across restarts */
    /* This prevents timing issues when ULP is restarted frequently */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* Initialize first time flag check for ADC */
    move r1, sample_count
    ld r2, r1, 0
    jump first_run_init, eq  // If sample_count is 0, do initialization
#endif
    jump main_loop

first_run_init:
#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
    /* First run - initialize ALL variables */
    move r1, sample_count
    move r2, 0
    st r2, r1, 0
    
    move r1, prev_result_idx
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
    
#if ULP_DEBUG_WAKEUPS_ENABLED
    move r1, debug_counter
    move r2, 0
    st r2, r1, 0
#endif

#if CONFIG_ULP_BUTTON_ENABLED
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
#endif

main_loop:
#if CONFIG_ULP_BUTTON_ENABLED
button_check:
    /* Read button state (0 = pressed, 1 = released) */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + BUTTON_RTC_IO, 1)
    
    /* Store raw button state for debugging */
    move r2, button_last_result    /* r2 = address of button_last_result */
    st r0, r2, 0                   /* Store GPIO state (r0 from READ_RTC_REG) */

    /* Check if button is pressed (GPIO = 0) */
    jump button_not_pressed, eq   /* If r0 == 0, button is pressed */

    /* Button is released - reset counter if it was non-zero */
    move r1, button_press_counter
    ld r2, r1, 0
    move r3, 0
    sub r3, r2, r3  /* Check if counter > 0 */
    jump button_counter_nonzero, ov /* If counter > 0 */
    jump check_adc_timing /* Counter is zero, no reset needed */

button_counter_nonzero:
    move r2, 0
    st r2, r1, 0
    jump check_adc_timing

button_not_pressed:
    /* Button is pressed - increment counter */
    move r1, button_press_counter
    ld r2, r1, 0
    add r2, r2, 1
    st r2, r1, 0

    /* Check for long press */
    move r3, LONG_PRESS_THRESHOLD
    sub r3, r2, r3
    jump button_long_press_detected, eq
    
    jump check_adc_timing

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
    /* Set button wake reason */
    move r0, 1              /* ULP_BUTTON_WAKE_LONG_PRESS */
    
    /* Inline set_button_wake logic */
    move r1, wake_data
    ld r2, r1, 0
    
    /* For button wake, clear last wake reasons but preserve current->last shift */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    
    /* But clear the last ADC and button reasons (keep last source) */
    and r3, r3, ULP_WAKE_LAST_SOURCE_MASK  /* Only keep source bits */
    
    and r2, r2, 0xFF00      /* Clear current bits */
    or r2, r2, r3           /* Combine with cleared last reasons */
    
    /* Clear current button and source, set new values */
    and r2, r2, 0xFF1C  /* ~(ULP_WAKE_CURRENT_BUTTON_MASK | ULP_WAKE_CURRENT_SOURCE_MASK) */
    /* Set button source bit (bit 1) */
    or r2, r2, ULP_WAKE_SOURCE_BUTTON

    /* Set button reason bits (bits 5-7) */
    lsh r0, r0, ULP_WAKE_CURRENT_BUTTON_SHIFT
    or r2, r2, r0
    
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
    move r1, prev_result_idx       /* r1 = address of index var */
    ld r2, r1, 0                   /* r2 = index value (0-7) */
    
    /* 2. If buffer is full (sample_count >= 8), subtract old value from running sum */
    move r1, sample_count
    ld r3, r1, 0                   /* r3 = sample_count */
    move r1, ADC_HISTORY_SIZE
    sub r1, r3, r1                 /* Check if sample_count >= 8 */
    jump buffer_not_full, ov       /* Skip subtraction if not full */
    
    /* Buffer is full - subtract old value at current index from running sum */
    move r1, prev_result           /* r1 = array base */
    lsh r3, r2, 0                  /* r3 = index * 1 (word addressing) */
    add r1, r1, r3                 /* r1 = &prev_result[index] */
    ld r3, r1, 0                   /* r3 = old value at this index */
    move r1, running_sum
    ld r0, r1, 0                   /* r0 = current running_sum */
    sub r0, r0, r3                 /* running_sum -= old_value */
    st r0, r1, 0                   /* store updated sum */
    
buffer_not_full:
    /* 3. Add new ADC value to running sum */
    move r1, last_result
    ld r0, r1, 0                   /* r0 = new ADC value */
    move r1, running_sum
    ld r3, r1, 0                   /* r3 = current running_sum */
    add r3, r3, r0                 /* running_sum += new_value */
    st r3, r1, 0                   /* store updated sum */
    
    /* 4. Store new ADC value in array at current index */
    move r1, prev_result_idx
    ld r2, r1, 0                   /* r2 = current index */
    move r3, prev_result           /* r3 = array base */
    lsh r1, r2, 0                  /* r1 = index * 1 (word addressing) */
    add r3, r3, r1                 /* r3 = &prev_result[index] */
    move r1, last_result
    ld r0, r1, 0                   /* r0 = new ADC value */
    st r0, r3, 0                   /* Store in array */
    
    /* 5. Advance index: (index + 1) % 8 */
    move r1, prev_result_idx
    ld r2, r1, 0                   /* r2 = current index */
    add r2, r2, 1                  /* index + 1 */
    and r2, r2, (ADC_HISTORY_SIZE-1) /* wrap 0-7 */
    st r2, r1, 0                   /* store new index */
    
    /* 6. Update sample_count */
    move r1, sample_count
    ld r2, r1, 0
    add r2, r2, 1
    move r3, ADC_HISTORY_SIZE
    sub r0, r2, r3
    jump sample_full, ov
    st r2, r1, 0
    jump check_full
sample_full:
    move r2, ULP_ADC_HISTORY_SIZE
    st r2, r1, 0

check_full:
    /* Only process if we have at least 4 samples */
    move r1, sample_count
    ld r2, r1, 0
    move r3, 4
    sub r0, r2, r3
    jump adc_exit, ov
    
    jump adc_calc_running_avg

adc_calc_running_avg:
    /* Calculate running average: running_sum / 8 */
    /* This mimics the main processor's efficient approach */
    move r1, running_sum
    ld r2, r1, 0                   /* r2 = running_sum */
    
    /* Calculate average (divide by ULP_ADC_HISTORY_SHIFT) */
    rsh r2, r2, ULP_ADC_HISTORY_SHIFT  /* r2 = running_sum / 8 */

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

    /* Check rapid change: if absolute difference >= threshold, trigger wake */
    move r3, cum_change
    ld r0, r3, 0                   /* r0 = absolute difference */
    move r3, rapid_change_thr
    ld r1, r3, 0                   /* r1 = rapid_change_thr */
    
    /* Compare: difference - threshold */
    sub r0, r0, r1                 /* r0 = difference - threshold */
    jump adc_rapid_change_triggered, eq  /* Trigger if equal */
    jump adc_check_thresholds, ov  /* Continue if difference < threshold (overflow) */
    jump adc_rapid_change_triggered    /* Trigger if difference > threshold */

#endif

adc_check_thresholds:
    /* Check last_result < low_thr */
    move r3, last_result
    ld r0, r3, 0
    move r3, low_thr
    ld r1, r3, 0
    sub r0, r0, r1
    jump adc_low_threshold_triggered, ov

    jump adc_exit

adc_exit:
#if ULP_DEBUG_WAKEUPS_ENABLED
    /* Debug counter with periodic wakeup */
    move r1, debug_counter
    ld r2, r1, 0
    add r2, r2, 1
    st r2, r1, 0
    
    /* Check if we should trigger periodic debug wake */
    /* Wake after 10 ADC checks = 10 seconds (ADC checked once per second) */
    move r3, 10  /* 10 ADC checks = 10 seconds */
    sub r0, r2, r3
    jump debug_periodic_wake, eq
#endif
    
    jump sleep_cycle

debug_periodic_wake:
    /* Trigger wake using high threshold reason (since high threshold never actually triggers) */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Shift current to last */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    and r2, r2, 0xFF00      /* Clear current bits, keep last */
    or r2, r2, r3           /* Combine old current into last */
    
    /* Set debug wake reason using ADC high threshold (value 2) */
    and r2, r2, 0xFFE3      /* Clear current ADC and source bits (bits 0-4) */
    or r2, r2, 0x0009       /* ULP_WAKE_SOURCE_ADC (1) | (ULP_ADC_WAKE_HIGH_THR (2) << 2) = 1 + 8 = 9 */
    
    st r2, r1, 0
    jump wake_main_cpu

adc_low_threshold_triggered:
    /* Check if same as last wake reason */
    move r1, wake_data
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_ADC_MASK      /* Current ADC reason */
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    and r1, r2, ULP_WAKE_LAST_ADC_MASK         /* Last ADC reason */
    rsh r1, r1, ULP_WAKE_LAST_ADC_SHIFT
    sub r0, r3, r1
    jump adc_exit, eq       /* Skip if same reason */
    
    /* Set ADC wake reason */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Shift current to last */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    and r2, r2, 0xFF00      /* Clear current bits, keep last */
    or r2, r2, r3           /* Combine old current into last */
    
    /* Clear current ADC and source, set low threshold reason */
    and r2, r2, 0xFFE3      /* Clear bits 0-1 and 2-4 */
    or r2, r2, 0x0005       /* ULP_WAKE_SOURCE_ADC | (ULP_ADC_WAKE_LOW_THR << 2) */
    
    st r2, r1, 0
    jump wake_main_cpu

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
adc_rapid_change_triggered:
    /* Check if same as last wake reason */
    move r1, wake_data
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_ADC_MASK      /* Current ADC reason */
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    and r1, r2, ULP_WAKE_LAST_ADC_MASK         /* Last ADC reason */
    rsh r1, r1, ULP_WAKE_LAST_ADC_SHIFT
    sub r0, r3, r1
    jump adc_exit, eq       /* Skip if same reason */
    
    /* Set ADC wake reason */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Shift current to last */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    and r2, r2, 0xFF00      /* Clear current bits, keep last */
    or r2, r2, r3           /* Combine old current into last */
    
    /* Clear current ADC and source, set rapid change reason */
    and r2, r2, 0xFFE3      /* Clear bits 0-1 and 2-4 */
    or r2, r2, 0x000D       /* ULP_WAKE_SOURCE_ADC | (ULP_ADC_WAKE_RAPID_CHG << 2) */
    
    st r2, r1, 0
    jump wake_main_cpu
#endif

/* Common wakeup handler for both ADC and button */
wake_main_cpu:
    /* No need to reset wake_source anymore - it's in wake_data */
    /* No need to reset cum_change - it's recalculated each cycle */

#if ULP_DEBUG_WAKEUPS_ENABLED
    /* Reset debug counter for next wake cycle */
    // move r3, debug_counter
    // move r0, 0
    // st r0, r3, 0
#endif

    /* Wake up the main CPU */
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump sleep_cycle, eq /* Try again later if not ready */
    wake  /* Wake main CPU */

    /* IMPORTANT: Remove these two lines completely */
    /* WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0) */
    /* halt */

    jump sleep_cycle  /* Continue ULP operation */

sleep_cycle:
    /* Before sleeping, check if we had any wake source */
    move r1, wake_data
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_SOURCE_MASK
    jump no_wake_source, eq
    
    /* We have a wake source - proceed to sleep without clearing */
    jump do_wait
    
no_wake_source:
    /* No wake source detected this cycle - clear wake data */
    move r2, 0
    st r2, r1, 0
    
do_wait:
    wait CYCLE_WAIT_TICKS
    jump main_loop
