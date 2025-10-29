#include "ulp_hardware.h"

static const char *TAG = "ulp_hw";
RTC_DATA_ATTR bool ulp_hw_initialized = false;

/* Map ADC channel to GPIO pin number */
int adc_channel_to_gpio(adc_channel_t channel) {
    switch (channel) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        case ADC_CHANNEL_0: return GPIO_NUM_1;
        case ADC_CHANNEL_1: return GPIO_NUM_2;
        case ADC_CHANNEL_2: return GPIO_NUM_3;
        case ADC_CHANNEL_3: return GPIO_NUM_4;
        case ADC_CHANNEL_4: return GPIO_NUM_5;
        case ADC_CHANNEL_5: return GPIO_NUM_6;
        case ADC_CHANNEL_6: return GPIO_NUM_7;
        case ADC_CHANNEL_7: return GPIO_NUM_8;
        case ADC_CHANNEL_8: return GPIO_NUM_9;
        case ADC_CHANNEL_9: return GPIO_NUM_10;
#else
        case ADC_CHANNEL_0: return GPIO_NUM_36;
        case ADC_CHANNEL_1: return GPIO_NUM_37;
        case ADC_CHANNEL_2: return GPIO_NUM_38;
        case ADC_CHANNEL_3: return GPIO_NUM_39;
        case ADC_CHANNEL_4: return GPIO_NUM_32;
        case ADC_CHANNEL_5: return GPIO_NUM_33;
        case ADC_CHANNEL_6: return GPIO_NUM_34;
        case ADC_CHANNEL_7: return GPIO_NUM_35;
#endif
        default:
            ELOG(TAG, "Unsupported ADC channel %d", channel);
            return -1;
    }
}

void adc_ulp_init_rtc_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }
    // Configure button GPIO for ULP use
    rtc_gpio_init(rtc_gpio);
    rtc_gpio_set_direction(rtc_gpio, RTC_GPIO_MODE_INPUT_ONLY);
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
    if (rtc_gpio >= 34 && rtc_gpio <= 39) {
#else
    if (rtc_gpio >= 34 && rtc_gpio <= 48) {
#endif
        goto end;
    }
    switch(rtc_gpio) {
#ifdef CONFIG_ULP_BUTTON_ENABLED
        case CONFIG_ULP_BUTTON_GPIO:
            rtc_gpio_pullup_en(rtc_gpio);
            rtc_gpio_pulldown_dis(rtc_gpio);
            break;
#endif
        default:
            rtc_gpio_pulldown_dis(rtc_gpio);
            rtc_gpio_pullup_dis(rtc_gpio);
            break;
    }
    end:
    DLOG(TAG, "GPIO pin %d configured for RTC.", rtc_gpio);
}


void adc_ulp_uninit_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }

    /* Deinit RTC mode first */
    rtc_gpio_deinit(rtc_gpio);

    /* GPIO 34-39 on ESP32 are input-only - skip reset and hold operations */
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
    if (rtc_gpio >= 34 && rtc_gpio <= 39) {
#else
    if (rtc_gpio >= 34 && rtc_gpio <= 48) {
#endif
        return;
    }

    /* Reset pin to default state (safe for output-capable GPIOs) */
    gpio_reset_pin(rtc_gpio);
    gpio_hold_dis(rtc_gpio);
    rtc_gpio_hold_dis(rtc_gpio);

    DLOG(TAG, "GPIO pin %d cleared", rtc_gpio);
}

void configure_adc_pad(void)
{
    FUNC_ENTRY(TAG);

    int adc_gpio = adc_channel_to_gpio(_ADC_CHANNEL_0);
    if (adc_gpio == -1) {
        return;
    }

    // Configure ADC pad for ULP use
    adc_ulp_init_rtc_pin(adc_gpio);

    ILOG(TAG, "ADC GPIO%d (channel %d) configured for ULP", adc_gpio, _ADC_CHANNEL_0);
}

void adc_ulp_init_pins(void)
{
    FUNC_ENTRY(TAG);
    configure_adc_pad();
#ifdef CONFIG_ULP_BUTTON_ENABLED
    adc_ulp_init_rtc_pin(CONFIG_ULP_BUTTON_GPIO);
#endif
}

void adc_ulp_uninit_pins(void)
{
    FUNC_ENTRY(TAG);
#ifdef CONFIG_ULP_BUTTON_ENABLED
    adc_ulp_uninit_pin(CONFIG_ULP_BUTTON_GPIO);
#endif
}

/**
 * Initialize ULP ADC hardware for battery monitoring
 * Can be called multiple times - exits early if already initialized
 * Only uses locking if main ADC is initialized (lock exists)
 */
esp_err_t init_ulp_adc(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;

    /* Exit early if already initialized */
    if (ulp_hw_initialized) {
        DLOG(TAG, "ULP ADC already initialized, skipping");
        return ESP_OK;
    }

    /* Try to acquire lock if ADC is initialized, otherwise just proceed */
    bool locked = adc_lock(1000);
    if (!locked) {
        DLOG(TAG, "ADC lock not available (main ADC not initialized), proceeding without lock");
    }

    /* Initialize ULP ADC hardware */
    ulp_adc_cfg_t adc_cfg = {
        .adc_n = _ADC_UNIT_0,     // Use same unit as regular ADC
        .channel = _ADC_CHANNEL_0, // Use same channel as regular ADC
        .atten = _ADC_ATTEN,      // Use same attenuation as regular ADC
        .width = _ADC_BITWIDTH,   // Use same bitwidth as regular ADC (only for ADC1)
        .ulp_mode = ADC_ULP_MODE_FSM, // Explicitly specify FSM mode for ESP32 (not RISC-V)
    };

    err = ulp_adc_init(&adc_cfg);
    if (err != ESP_OK) {
        ELOG(TAG, "ULP ADC init failed: %s", esp_err_to_name(err));
        if (locked) adc_unlock();
        return err;
    }

    /* Configure GPIO pins for ULP use */
    adc_ulp_init_pins();

    ulp_hw_initialized = true;
    // ILOG(TAG, "ULP ADC hardware initialized");

    if (locked) adc_unlock();
    return ESP_OK;
}

/**
 * Deinitialize ULP ADC hardware
 * Can be called multiple times - exits early if not initialized
 * Only uses locking if main ADC is initialized (lock exists)
 */
void deinit_ulp_adc(void) {
    FUNC_ENTRY(TAG);

    /* Exit early if not initialized */
    if (!ulp_hw_initialized) {
        DLOG(TAG, "ULP ADC not initialized, nothing to deinit");
        return;
    }

    /* Try to acquire lock if ADC is initialized, otherwise just proceed */
    bool locked = adc_lock(1000);
    if (!locked) {
        DLOG(TAG, "ADC lock not available (main ADC not initialized), proceeding without lock");
    }

    /* Clear flag first to prevent re-entry if deinit fails */
    ulp_hw_initialized = false;

    esp_err_t err = ulp_adc_deinit();
    if (err != ESP_OK) {
        /* Log error but continue cleanup - don't restore flag */
        ELOG(TAG, "ULP ADC deinit failed: %s", esp_err_to_name(err));
    } else {
        ILOG(TAG, "ULP ADC hardware deinitialized");
    }

    /* Uninitialize the ADC GPIO pin from RTC mode (safe even if deinit failed) */
    int adc_gpio = adc_channel_to_gpio(_ADC_CHANNEL_0);
    if (adc_gpio != -1) {
        adc_ulp_uninit_pin(adc_gpio);
    }

    if (locked) adc_unlock();
}