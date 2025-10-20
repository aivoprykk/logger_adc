#include "myadc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "esp_adc/adc_oneshot.h"
#include "ulp_adc.h"

#include "ulp_myadc.h"
#include "ulp/my_ulp_config.h"

extern const uint8_t ulp_myadc_bin_start[] asm("_binary_ulp_myadc_bin_start");
extern const uint8_t ulp_myadc_bin_end[]   asm("_binary_ulp_myadc_bin_end");

void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_myadc_bin_start,
            (ulp_myadc_bin_end - ulp_myadc_bin_start) / sizeof(uint32_t));
    if(err) {
        printf("Failed to load ULP binary: %s\n", esp_err_to_name(err));
        return;
    }

    ulp_adc_cfg_t cfg = {
        .adc_n    = EXAMPLE_ADC_UNIT,
        .channel  = EXAMPLE_ADC_CHANNEL,
        .width    = EXAMPLE_ADC_WIDTH,
        .atten    = EXAMPLE_ADC_ATTEN,
        .ulp_mode = ADC_ULP_MODE_FSM,
    };

    if(ulp_adc_init(&cfg)) {
        printf("Failed to init ULP ADC\n");
        return;
    }

    ulp_low_thr = EXAMPLE_ADC_LOW_THRESHOLD;
    ulp_high_thr = EXAMPLE_ADC_HIGH_THRESHOLD;

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 20000);

#if CONFIG_IDF_TARGET_ESP32
    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    rtc_gpio_isolate(GPIO_NUM_39);
#endif // CONFIG_IDF_TARGET_ESP32

    // esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

void start_ulp_program(void)
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        printf("Failed to start ULP program: %s\n", esp_err_to_name(err));
        return;
    }
}

void take_measurement(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");
        init_ulp_program();
    } else {
        printf("Deep sleep wakeup\n");
        printf("ULP did %"PRIu32" measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        printf("Thresholds:  low=%"PRIu32"  high=%"PRIu32"\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        printf("Value=%"PRIu32" was %s threshold\n", ulp_last_result,
                ulp_last_result < ulp_low_thr ? "below" : "above");
    }
    printf("Entering deep sleep\n\n");
    start_ulp_program();
    if( esp_sleep_enable_ulp_wakeup() ) {
        printf("esp_sleep_enable_ulp_wakeup failed\n");
        return;
    }
}