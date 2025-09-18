/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "../adc_config.h"

	.set adc_channel, ULP_ADC_CHANNEL
	.set adc_unit, ULP_ADC_UNIT

	/* Configure the number of ADC samples to average on each measurement.
	   For convenience, make it a power of 2. */
	.set adc_oversampling_factor_log, 2
	.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	/* Low threshold of ADC reading.
	   Set by the main program. */
	.global low_thr
low_thr:
	.long 0

	/* High threshold of ADC reading.
	   Set by the main program. */
	.global high_thr
high_thr:
	.long 0

	/* Counter of measurements done */
	.global sample_counter
sample_counter:
	.long 0

	.global last_result
last_result:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* increment sample counter */
	move r3, sample_counter
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0

	/* do measurements using ADC */
	/* r0 will be used as accumulator */
	move r0, 0
	/* initialize the loop counter */
	stage_rst
measure:
	adc r1, 0, 7  /* ADC unit 0, channel 7 */
	add r0, r0, r1
	/* increment loop counter and check exit condition */
	stage_inc 1
	jumps measure, 4, lt

	/* divide accumulator by adc_oversampling_factor.
	   Since it is chosen as a power of two, use right shift */
	rsh r0, r0, 2  /* Divide by 4 (2^2) */
	/* averaged value is now in r0; store it into last_result */
	move r3, last_result
	st r0, r3, 0

	/* Also store the result in RTC_SLOW_MEM[0] for C code access */
	move r3, 0x50000000  /* RTC_SLOW_MEM base address */
	st r0, r3, 0        /* Store to RTC_SLOW_MEM[0] */

	/* compare with low_thr; wake up if value < low_thr */
	move r3, low_thr
	ld r3, r3, 0
	sub r3, r0, r3
	jump wake_up, ov

	/* compare with high_thr; wake up if value > high_thr */
	move r3, high_thr
	ld r3, r3, 0
	sub r3, r3, r0
	jump wake_up, ov

	/* value within range, end the program */
	.global exit
exit:
	halt

	.global wake_up
wake_up:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

	/* Wake up the SoC, end program */
	wake
	/* NOTE: Do NOT disable ULP timer - let it continue running periodically */
	halt