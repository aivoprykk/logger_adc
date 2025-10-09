/* 
 * Simplified ULP program for ADC battery monitoring and button long press detection
 * Focused on stability and avoiding immediate wakeups
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "../ulp_sensors_config.h"

#include "battery_sensing.S"

/* Button configuration from Kconfig */
.set BUTTON_RTC_IO, ULP_BUTTON_RTC_IO  
.set LONG_PRESS_THRESHOLD, ULP_LONG_PRESS_CYCLES

/* Timing configuration */
.set CYCLE_WAIT_TICKS, (8000 * ULP_CYCLE_TIME_MS / 1000)  // Convert ms to ULP ticks

/* Global variables for button monitoring */
.global button_press_counter
button_press_counter:
    .long 0
.global button_wake_reason
button_wake_reason:
    .long 0  // 0: no press, 1: long press detected

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
    jump adc_measure_and_check, eq  // Jump to ADC if r3 == 0 (time for ADC check)
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