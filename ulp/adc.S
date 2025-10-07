#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "../adc_config.h"
.set adc_channel, ULP_ADC_CHANNEL  // GPIO 35 (ADC1 channel 7)
.set adc_oversampling_factor_log, 2
.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

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
.global wake_reason
wake_reason:
    .long 0  // 1: low_thr, 2: high_thr, 3: rapid_change
.global last_wake_reason
last_wake_reason:
    .long 0  // Persisted reason to suppress repeated wakeups

.text
.global entry
entry:
    /* Delay to stabilize ADC */
    move r0, 20000  // ~20ms delay at 8MHz
delay:
    sub r0, r0, 1
    jump delay, ov
    /* ADC measurement */
    move r0, 0
    stage_rst
measure:
    adc r1, 0, adc_channel + 1
    add r0, r0, r1
    stage_inc 1
    jumps measure, adc_oversampling_factor, lt
    rsh r0, r0, adc_oversampling_factor_log
    move r3, last_result
    st r0, r3, 0

    /* Validate ADC reading */
    move r1, r0
    sub r1, r1, 200  // Ensure last_result >= 200
    jump exit, ov  // r1 < 200 (underflow)

    /* Update circular buffer */
    move r3, prev_result_idx
    ld r1, r3, 0
    move r2, prev_result
    add r2, r2, r1
    st r0, r2, 0

    /* Advance circular index */
    add r1, r1, 1
    move r2, r1
    sub r2, r2, 8
    jump store_idx, ov  // stay within 0..7 while buffer not wrapped
    move r1, 0
store_idx:
    move r3, prev_result_idx
    st r1, r3, 0

    /* Track how many samples we have collected (saturate at 8) */
    move r3, sample_count
    ld r1, r3, 0
    add r1, r1, 1
    move r2, r1
    sub r2, r2, 8
    jump sample_count_store, ov
    move r1, 8
sample_count_store:
    st r1, r3, 0

    /* Skip cum_change until buffer is full */
    move r3, sample_count
    ld r1, r3, 0
    sub r0, r1, 8
    jump check_thresholds, ov

calc_running_avg:
    /* Calculate running average */
    move r3, prev_result
    move r2, 0
    ld r0, r3, 0
    add r2, r2, r0
    ld r0, r3, 1
    add r2, r2, r0
    ld r0, r3, 2
    add r2, r2, r0
    ld r0, r3, 3
    add r2, r2, r0
    ld r0, r3, 4
    add r2, r2, r0
    ld r0, r3, 5
    add r2, r2, r0
    ld r0, r3, 6
    add r2, r2, r0
    ld r0, r3, 7
    add r2, r2, r0
    rsh r2, r2, 3  // r2 = running_avg = sum / 8

    /* Calculate cum_change: |last_result - running_avg| */
    move r3, last_result
    ld r0, r3, 0
    sub r1, r0, r2       // tentative diff = last_result - running_avg
    jump pos_diff, ov    // if overflow, diff was negative -> compute opposite order
    jump store_diff
pos_diff:
    sub r1, r2, r0
store_diff:
    move r3, cum_change
    st r1, r3, 0

    /* Check cum_change >= rapid_change_thr */
    move r3, cum_change
    ld r0, r3, 0
    move r3, rapid_change_thr
    ld r1, r3, 0
    sub r1, r1, r0  // rapid_change_thr - cum_change
    jump wake_rapid, eq  // cum_change == rapid_change_thr
    jump wake_rapid, ov  // cum_change > rapid_change_thr

check_thresholds:
    /* Check last_result < low_thr */
    move r3, last_result
    ld r0, r3, 0
    move r3, low_thr
    ld r1, r3, 0
    sub r0, r0, r1  // last_result - low_thr
    jump wake_low, ov  // last_result < low_thr

    /* Check last_result > high_thr */
    move r3, last_result
    ld r0, r3, 0
    move r3, high_thr
    ld r1, r3, 0
    sub r0, r1, r0  // high_thr - last_result
    jump wake_high, ov  // last_result > high_thr

.global exit
exit:
    halt

wake_low:
    /* Suppress repeated wakeups for the same low threshold trigger */
    move r3, last_wake_reason
    ld r2, r3, 0
    move r1, 1
    sub r2, r2, r1
    jump skip_wake_low, eq
    move r3, wake_reason
    move r0, 1
    st r0, r3, 0
    move r3, last_wake_reason
    st r0, r3, 0
    jump wake_up

skip_wake_low:
    jump exit

wake_high:
    /* Suppress repeated wakeups for the same high threshold trigger */
    move r3, last_wake_reason
    ld r2, r3, 0
    move r1, 2
    sub r2, r2, r1
    jump skip_wake_high, eq
    move r3, wake_reason
    move r0, 2
    st r0, r3, 0
    move r3, last_wake_reason
    st r0, r3, 0
    jump wake_up

skip_wake_high:
    jump exit

wake_rapid:
    /* Suppress repeated wakeups for the same rapid-change trigger */
    move r3, last_wake_reason
    ld r2, r3, 0
    move r1, 3
    sub r2, r2, r1
    jump skip_wake_rapid, eq
    move r3, wake_reason
    move r0, 3
    st r0, r3, 0
    move r3, last_wake_reason
    st r0, r3, 0
    jump wake_up

skip_wake_rapid:
    jump exit

.global wake_up
wake_up:
    /* Reset cum_change and prev_result_idx */
    move r3, cum_change
    move r0, 0
    st r0, r3, 0
    move r3, prev_result_idx
    st r0, r3, 0
    move r3, sample_count
    st r0, r3, 0
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump exit, eq
    wake
    WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
    halt