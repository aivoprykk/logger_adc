#ifndef A7081C0B_F40A_45EC_BD7E_1F8762A32B76
#define A7081C0B_F40A_45EC_BD7E_1F8762A32B76

#include "adc_private.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "ulp_adc.h"
#include "esp_err.h"

/**
 * @brief ULP Hardware Abstraction Layer
 *
 * This module provides low-level hardware abstraction for ULP ADC operations,
 * including GPIO pin management and ULP ADC hardware initialization.
 */

/**
 * @brief Map ADC channel to GPIO pin number
 *
 * @param channel ADC channel to map
 * @return GPIO pin number, or -1 if unsupported
 */
int adc_channel_to_gpio(adc_channel_t channel);

/**
 * @brief Configure GPIO pin for ULP/RTC use
 *
 * @param rtc_gpio RTC GPIO pin number
 */
void adc_ulp_init_rtc_pin(int rtc_gpio);

/**
 * @brief Deconfigure GPIO pin from RTC mode
 *
 * @param rtc_gpio RTC GPIO pin number
 */
void adc_ulp_uninit_pin(int rtc_gpio);

/**
 * @brief Initialize all GPIO pins required for ULP operation
 */
void adc_ulp_init_pins(void);

/**
 * @brief Deinitialize all GPIO pins from ULP operation
 */
void adc_ulp_uninit_pins(void);

/**
 * @brief Configure ADC pad for ULP use
 */
void configure_adc_pad(void);

/**
 * @brief Initialize ULP ADC hardware
 *
 * Can be called multiple times - exits early if already initialized.
 * Only uses locking if main ADC is initialized (lock exists).
 *
 * @return ESP_OK on success, error code otherwise
 */
// exported to adc.h
// esp_err_t init_ulp_adc(void);

/**
 * @brief Deinitialize ULP ADC hardware
 *
 * Can be called multiple times - exits early if not initialized.
 * Only uses locking if main ADC is initialized (lock exists).
 */
// exported to adc.h
// void deinit_ulp_adc(void);

#endif /* A7081C0B_F40A_45EC_BD7E_1F8762A32B76 */
