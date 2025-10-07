/* 
 * Simplified ULP program for ADC battery monitoring and button long press detection
 * Focused on stability and avoiding immediate wakeups
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "../ulp_sensors_config.h"

/* Configuration from Kconfig */
.set adc_channel, ULP_ADC_CHANNEL
.set adc_oversampling_factor_log, 2
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

/* Button configuration from Kconfig */
.set BUTTON_RTC_IO, ULP_BUTTON_RTC_IO  
.set LONG_PRESS_THRESHOLD, ULP_LONG_PRESS_CYCLES

/* Timing configuration */
.set CYCLE_WAIT_TICKS, (8000 * ULP_CYCLE_TIME_MS / 1000)  // Convert ms to ULP ticks

/* Global variables for ADC monitoring */
.bss
.global low_thr
low_thr:
    .long 0
.global high_thr
high_thr:
    .long 0
.global rapid_change_thr
rapid_change_thr:
    .long 0
.global cum_change
cum_change:
    .long 0
.global last_result
last_result:
    .long 0
.global prev_result
prev_result:
    .long 0  // prev_result[0]
    .long 0  // prev_result[1]
    .long 0  // prev_result[2]
    .long 0  // prev_result[3]
    .long 0  // prev_result[4]
    .long 0  // prev_result[5]
    .long 0  // prev_result[6]
    .long 0  // prev_result[7]
.global prev_result_idx
prev_result_idx:
    .long 0
.global sample_count
sample_count:
    .long 0
.global adc_wake_reason
adc_wake_reason:
    .long 0  // 1: low_thr, 2: high_thr, 3: rapid_change
.global last_wake_reason
last_wake_reason:
    .long 0  // Persisted reason to suppress repeated wakeups

/* Global variables for button monitoring */
.global button_press_counter
button_press_counter:
    .long 0
.global button_wake_reason
button_wake_reason:
    .long 0  // 0: no press, 1: long press detected

/* Wake up source flags */
.global wake_source
wake_source:
    .long 0  // Bit 0: ADC, Bit 1: Button

.text
.global entry
entry:
    /* Initialize - clear wake source */
    move r1, wake_source
    move r2, 0
    st r2, r1, 0
    
    /* Initialize first time flag check */
    move r1, sample_count
    ld r2, r1, 0
    jump first_run_init, eq  // If sample_count is 0, do initialization

normal_operation:
    /* === BUTTON CHECK === */
    /* Read button state (0 = pressed, 1 = released) */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + BUTTON_RTC_IO, 1)
    
    move r1, button_press_counter
    ld r2, r1, 0
    
    /* If GPIO is 0, button is pressed */
    jump button_pressed, eq
    
    /* Button not pressed - reset counter */
    move r2, 0
    st r2, r1, 0
    jump check_adc_timing

button_pressed:
    /* Button is pressed - increment counter */
    add r2, r2, 1
    st r2, r1, 0
    
    /* Check if reached long press threshold */
    move r3, LONG_PRESS_THRESHOLD
    sub r3, r2, r3
    jump button_long_press_detected, ov

check_adc_timing:

    /* === ADC CHECK (PERIODIC) === */
    /* Only check ADC every 8th cycle (once per second with 125ms cycles) */
    move r1, sample_count
    ld r2, r1, 0
    add r2, r2, 1
    st r2, r1, 0
    
    and r3, r2, 7  // Check if r2 % 8 == 0
    jump adc_measure, eq  // Jump to ADC if r3 == 0 (time for ADC check)
    jump sleep_cycle  // Skip ADC if not time yet

first_run_init:
    /* First run - initialize variables and skip checks */
    move r1, sample_count
    move r2, 1
    st r2, r1, 0
    
    /* Initialize other variables */
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
    
    move r1, last_wake_reason
    move r2, 0
    st r2, r1, 0
    
    jump sleep_cycle

adc_measure:

    /* === ADC MEASUREMENT === */
    /* Delay to stabilize ADC */
    move r0, 1600  // ~200 cycles delay
adc_delay:
    sub r0, r0, 1
    jump adc_delay, ov

    /* Read ADC with oversampling */
    move r3, 0
    stage_rst
adc_read_loop:
    adc r0, 0, adc_channel + 1
    add r3, r3, r0
    stage_inc 1
    jumps adc_read_loop, adc_oversampling_factor, lt
    
    rsh r3, r3, adc_oversampling_factor_log
    
    /* Store current reading */
    move r1, last_result
    st r3, r1, 0
    
    /* Get previous result for comparison */
    move r1, prev_result
    move r2, prev_result_idx
    ld r0, r2, 0
    add r0, r1, r0  // Use direct offset (ULP works with word addresses)
    ld r2, r0, 0  // r2 = previous result
    
    /* Store current result in circular buffer */
    st r3, r0, 0
    
    /* Update circular buffer index */
    move r1, prev_result_idx
    ld r0, r1, 0
    add r0, r0, 1
    and r0, r0, 7  // Wrap around at 8
    st r0, r1, 0
    
    /* === THRESHOLD CHECKS === */
    /* Check low threshold */
    move r1, low_thr
    ld r1, r1, 0
    sub r1, r3, r1  // r3 - low_thr
    jump low_threshold_triggered, ov  // Jump if underflow (r3 < low_thr)
    
    /* Check high threshold */
    move r1, high_thr
    ld r1, r1, 0
    sub r1, r3, r1  // r3 - high_thr
    jump high_threshold_triggered, ov  // Jump if no underflow (r3 >= high_thr)
    
    /* Check rapid change */
    sub r1, r3, r2  // current - previous
    jump positive_diff, ov  // Jump if no underflow (current >= previous)
    /* Make positive if negative */
    sub r1, r2, r3  // previous - current
    
positive_diff:
    /* Check against rapid change threshold */
    move r0, rapid_change_thr
    ld r0, r0, 0
    sub r0, r1, r0
    jump rapid_change_triggered, ov  // Jump if no underflow (r1 >= threshold)
    
    jump sleep_cycle

button_long_press_detected:
    /* Set button wake reason */
    move r1, button_wake_reason
    move r2, 1
    st r2, r1, 0
    
    /* Set wake source bit 1 (button) */
    move r1, wake_source
    ld r2, r1, 0
    or r2, r2, 2  // Set bit 1
    st r2, r1, 0
    
    /* Reset button counter */
    move r1, button_press_counter
    move r2, 0
    st r2, r1, 0
    jump wake_main_cpu

low_threshold_triggered:
    /* Avoid repeated wakeups for same condition */
    move r1, last_wake_reason
    ld r2, r1, 0
    sub r2, r2, 1  // Compare with reason 1
    jump sleep_cycle, eq  // Skip if already triggered
    
    /* Set ADC wake reason and wake source */
    move r1, adc_wake_reason
    move r2, 1
    st r2, r1, 0
    
    move r1, last_wake_reason  
    st r2, r1, 0
    
    move r1, wake_source
    ld r2, r1, 0
    or r2, r2, 1  // Set bit 0 (ADC)
    st r2, r1, 0
    jump wake_main_cpu

high_threshold_triggered:
    /* Avoid repeated wakeups for same condition */
    move r1, last_wake_reason
    ld r2, r1, 0
    sub r2, r2, 2  // Compare with reason 2
    jump sleep_cycle, eq  // Skip if already triggered
    
    /* Set ADC wake reason and wake source */
    move r1, adc_wake_reason
    move r2, 2
    st r2, r1, 0
    
    move r1, last_wake_reason
    st r2, r1, 0
    
    move r1, wake_source
    ld r2, r1, 0
    or r2, r2, 1  // Set bit 0 (ADC)
    st r2, r1, 0
    jump wake_main_cpu

rapid_change_triggered:
    /* Avoid repeated wakeups for same condition */
    move r1, last_wake_reason
    ld r2, r1, 0
    sub r2, r2, 3  // Compare with reason 3
    jump sleep_cycle, eq  // Skip if already triggered
    
    /* Set ADC wake reason and wake source */
    move r1, adc_wake_reason
    move r2, 3
    st r2, r1, 0
    
    move r1, last_wake_reason
    st r2, r1, 0
    
    move r1, wake_source
    ld r2, r1, 0
    or r2, r2, 1  // Set bit 0 (ADC)
    st r2, r1, 0
    jump wake_main_cpu

sleep_cycle:
    /* Sleep and repeat */
    wait CYCLE_WAIT_TICKS
    jump entry

wake_main_cpu:
    /* Reset wake source before waking to prevent loops */
    move r1, wake_source
    move r2, 0
    st r2, r1, 0
    
    /* Wake up the main CPU */
    wake
    
    /* Stop the ULP program after wake to prevent immediate re-execution */
    halt