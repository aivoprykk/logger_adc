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
/* ULP memory is 32-bit word addressed - must use .long for reliable access */
/* For values that fit in 16 bits, we use only lower 16 bits of the 32-bit word */
.global wake_data
wake_data:
    .long 0  /* 32-bit: packed wake source/reason data */

#if CONFIG_ULP_BUTTON_ENABLED
.global button_press_counter
button_press_counter:
    .long 0  /* ULP st instruction writes 32-bit, must use .long */
.global button_last_result
button_last_result:
    .long 0  /* ULP st instruction writes 32-bit, must use .long */
#endif

.global low_thr
low_thr:
    .long 0  /* 32-bit word, only lower 12 bits used (ADC threshold max 4095) */
.global last_result
last_result:
    .long 0  /* 32-bit word, only lower 12 bits used (ADC reading max 4095) */

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
.global rapid_change_thr
rapid_change_thr:
    .long 0  /* 32-bit word, only lower 16 bits used (~120) */
.global cum_change
cum_change:
    .long 0  /* 32-bit word, only lower 16 bits used */
.global prev_result
prev_result: 
    .space (ADC_HISTORY_SIZE * 4)  /* 8 elements * 4 bytes, only lower 12 bits used */
.global prev_result_idx
prev_result_idx:
    .long 0  /* 32-bit word, only lower 16 bits used (index 0-7) */
.global sample_count
sample_count:
    .long 0  /* 32-bit word, only lower 16 bits used (count 0-8) */
.global running_sum
running_sum:
    .long 0  /* 32-bit: full word used (sum can be 8*4095=32760) */
#endif

.global cycle_count
cycle_count:
    .long 0  /* 32-bit: full word used (can grow large) */

.text
.global entry
entry:
    /* DON'T clear wake_data here - main CPU clears it via adc_ulp_clear_wake_sources() */
    /* This preserves wake_data so main CPU can read it after ULP wake */

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

#if CONFIG_ULP_BUTTON_ENABLED
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
#endif

    /* Clear wake_data including debug button counter (bits 16-17) */
    move r1, wake_data
    move r2, 0
    st r2, r1, 0

main_loop:
#if CONFIG_ULP_BUTTON_ENABLED
button_check:
    /* Read button state (0 = pressed with pull-up, 1 = released) */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + BUTTON_RTC_IO, 1)
    
    /* Store raw button state for debugging */
    move r2, button_last_result    /* r2 = address of button_last_result */
    st r0, r2, 0                   /* Store GPIO state (r0 from READ_RTC_REG) */

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
    /* Check if we already have a wake event pending - if so, FREEZE all ADC processing */
    move r1, wake_data
    ld r2, r1, 0
    and r3, r2, ULP_WAKE_CURRENT_SOURCE_MASK  /* Check bits 0-1 for wake source */
    move r0, 0
    sub r0, r3, r0  /* Check if r3 != 0 */
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
    
    /* 6. Update sample_count - cap at ADC_HISTORY_SIZE */
    move r1, sample_count
    ld r2, r1, 0
    move r3, ADC_HISTORY_SIZE
    sub r0, r2, r3
    jump sample_full, eq  /* If already at max (8), don't increment */
    
    /* Increment sample_count */
    add r2, r2, 1
    st r2, r1, 0
    jump check_full
    
sample_full:
    /* Already at max size, keep it there */
    move r2, ADC_HISTORY_SIZE
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

adc_frozen:
    /* ADC is frozen because wake is already set - just sleep */
    jump sleep_cycle

adc_exit:
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
    /* Check if same as last wake reason (anti-spam) */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Extract last ADC reason */
    and r1, r2, ULP_WAKE_LAST_ADC_MASK         /* Last ADC reason */
    rsh r1, r1, ULP_WAKE_LAST_ADC_SHIFT
    
    /* If last reason is NONE (0), always wake - this is first wake */
    jump adc_low_set_wake, eq
    
    /* Extract current ADC reason */
    and r3, r2, ULP_WAKE_CURRENT_ADC_MASK      /* Current ADC reason */
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    
    /* Compare current with last */
    sub r0, r3, r1
    jump adc_exit, eq       /* Skip only if same non-zero reason */
    
adc_low_set_wake:
    /* Set ADC wake reason */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Shift current to last */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    and r2, r2, 0xFF00      /* Clear current bits, keep last */
    or r2, r2, r3           /* Combine old current into last */
    
    /* Clear current ADC and source bits (0-4) */
    and r2, r2, 0xFFE0      /* Clear bits 0-4 */
    
    /* Set source = ADC (bit 0) */
    or r2, r2, ULP_WAKE_SOURCE_ADC  /* Set bit 0 = 1 */
    
    /* Set ADC reason = LOW_THR (1) in bits 2-4 */
    move r0, ULP_ADC_WAKE_LOW_THR   /* r0 = 1 */
    lsh r0, r0, 2                    /* r0 = 1 << 2 = 4 */
    or r2, r2, r0                    /* Set bits 2-4 */
    
    st r2, r1, 0
    jump wake_main_cpu

#if CONFIG_ULP_BATTERY_MONITORING_ENABLED
adc_rapid_change_triggered:
    /* Check if same as last wake reason (anti-spam) */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Extract last ADC reason */
    and r1, r2, ULP_WAKE_LAST_ADC_MASK         /* Last ADC reason */
    rsh r1, r1, ULP_WAKE_LAST_ADC_SHIFT
    
    /* If last reason is NONE (0), always wake - this is first wake */
    jump adc_rapid_set_wake, eq
    
    /* Extract current ADC reason */
    and r3, r2, ULP_WAKE_CURRENT_ADC_MASK      /* Current ADC reason */
    rsh r3, r3, ULP_WAKE_CURRENT_ADC_SHIFT
    
    /* Compare current with last */
    sub r0, r3, r1
    jump adc_exit, eq       /* Skip only if same non-zero reason */
    
adc_rapid_set_wake:
    /* Set ADC wake reason */
    move r1, wake_data
    ld r2, r1, 0
    
    /* Shift current to last */
    move r3, r2
    and r3, r3, 0x00FF      /* Get current bits (0-7) */
    lsh r3, r3, 8           /* Shift to last position (8-15) */
    and r2, r2, 0xFF00      /* Clear current bits, keep last */
    or r2, r2, r3           /* Combine old current into last */
    
    /* Clear current ADC and source bits (0-4) */
    and r2, r2, 0xFFE0      /* Clear bits 0-4 */
    
    /* Set source = ADC (bit 0) */
    or r2, r2, ULP_WAKE_SOURCE_ADC  /* Set bit 0 = 1 */
    
    /* Set ADC reason = RAPID_CHG (3) in bits 2-4 */
    move r0, ULP_ADC_WAKE_RAPID_CHG  /* r0 = 3 */
    lsh r0, r0, 2                     /* r0 = 3 << 2 = 12 */
    or r2, r2, r0                     /* Set bits 2-4 */
    
    st r2, r1, 0
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
    /* This ensures wake reason persists until main CPU processes it */
    wait CYCLE_WAIT_TICKS
    jump main_loop
