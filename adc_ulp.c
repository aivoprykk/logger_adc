#include "adc_private.h"

#if defined(CONFIG_ULP_COPROC_ENABLED)

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "hal/rtc_hal.h"
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_adc.h"
#include "ulp_common.h"
#include "ulp_common_defs.h"
#include "ulp_sensors_config.h"

static const char *TAG = "adc_ulp";

/* ULP binary references - updated for unified sensors */
extern const uint8_t ulp_unified_sensors_bin_start[] asm("_binary_ulp_unified_sensors_bin_start");
extern const uint8_t ulp_unified_sensors_bin_end[]   asm("_binary_ulp_unified_sensors_bin_end");

/* ADC ULP variables */
extern uint32_t ulp_low_thr, ulp_high_thr, ulp_rapid_change_thr, ulp_cum_change;
extern uint32_t ulp_last_result, ulp_prev_result[8], ulp_prev_result_idx, ulp_entry;
extern uint32_t ulp_adc_wake_reason, ulp_last_wake_reason;

/* Button ULP variables */
extern uint32_t ulp_button_press_counter, ulp_button_wake_reason;

/* Unified wake source */
extern uint32_t ulp_wake_source;

// ULP program symbols - from generated header  
// #include "ulp_logger_adc.h"

void configure_adc_pad(void)
{
    FUNC_ENTRY(TAG);
    // Map ADC channel to GPIO pin for ESP32
    int adc_gpio = -1;
    switch (_ADC_CHANNEL_0) {
        case ADC_CHANNEL_0: adc_gpio = GPIO_NUM_36; break;
        case ADC_CHANNEL_1: adc_gpio = GPIO_NUM_37; break;
        case ADC_CHANNEL_2: adc_gpio = GPIO_NUM_38; break;
        case ADC_CHANNEL_3: adc_gpio = GPIO_NUM_39; break;
        case ADC_CHANNEL_4: adc_gpio = GPIO_NUM_32; break;
        case ADC_CHANNEL_5: adc_gpio = GPIO_NUM_33; break;
        case ADC_CHANNEL_6: adc_gpio = GPIO_NUM_34; break;
        case ADC_CHANNEL_7: adc_gpio = GPIO_NUM_35; break;
        default:
            ELOG(TAG, "Unsupported ADC channel %d for ULP", _ADC_CHANNEL_0);
            return;
    }
    
    // Configure ADC pad for ULP use
    rtc_gpio_init(adc_gpio);
    rtc_gpio_set_direction(adc_gpio, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(adc_gpio);
    rtc_gpio_pulldown_dis(adc_gpio);

    ILOG(TAG, "ADC pad GPIO%d (channel %d) configured for ULP", adc_gpio, _ADC_CHANNEL_0);
}

#ifdef CONFIG_ULP_BUTTON_ENABLED
void configure_button_pad(void)
{
    FUNC_ENTRY(TAG);
    
    // Configure button GPIO for ULP use
    int button_gpio = CONFIG_ULP_BUTTON_GPIO;
    
    rtc_gpio_init(button_gpio);
    rtc_gpio_set_direction(button_gpio, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(button_gpio);  // Enable internal pullup for button
    rtc_gpio_pulldown_dis(button_gpio);
    
    ILOG(TAG, "Button pad GPIO%d (RTC_IO%d) configured for ULP", 
         button_gpio, CONFIG_ULP_BUTTON_RTC_IO);
}
#endif

/**
 * Initialize the ULP program for battery monitoring
 */
esp_err_t init_ulp_program(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;

    ILOG(TAG, "Initializing ULP program...");
    if(adc_lock(1000)) {
        adc_unlock();
    }
    // Reset and prepare ULP
    ulp_timer_stop();
    vTaskDelay(pdMS_TO_TICKS(50));

   // First, let's verify the unified ULP binary was loaded correctly
    const size_t ulp_prog_size_bytes = ulp_unified_sensors_bin_end - ulp_unified_sensors_bin_start;
    const size_t ulp_prog_size_words = ulp_prog_size_bytes / sizeof(uint32_t);

    err = ulp_load_binary(0, ulp_unified_sensors_bin_start, ulp_prog_size_words);
    if (err != ESP_OK) {
        ELOG(TAG, "Failed to load unified ULP program: %s", esp_err_to_name(err));
        goto error;
    }

    ulp_adc_cfg_t adc_cfg = {
        .adc_n = _ADC_UNIT_0,     // Use same unit as regular ADC
        .channel = _ADC_CHANNEL_0, // Use same channel as regular ADC  
        .width = _ADC_BITWIDTH,   // Use same bitwidth as regular ADC (only for ADC1)
        .atten = _ADC_ATTEN,      // Use same attenuation as regular ADC
        .ulp_mode = ADC_ULP_MODE_FSM, // Explicitly specify FSM mode for ESP32 (not RISC-V)
    };

    // Initialize ULP ADC
    err = ulp_adc_init(&adc_cfg);
    if (err != ESP_OK) {
        ELOG(TAG, "ULP ADC init failed: %s", esp_err_to_name(err));
        return err;
    }

    configure_adc_pad();
    
#ifdef CONFIG_ULP_BUTTON_ENABLED
    configure_button_pad();
#endif

    // Initialize ADC thresholds
    ulp_low_thr = ADC_LOW_TRESHOLD;
    ulp_high_thr = ADC_HIGH_TRESHOLD;
    ulp_rapid_change_thr = ADC_RAPID_CHANGE_TRESHOLD;
    
    // Initialize button and wake source variables
    ulp_button_press_counter = 0;
    ulp_button_wake_reason = 0;
    ulp_wake_source = 0;

    ILOG(TAG, "ULP program initialized low_thr=%lu, high_thr=%lu, rapid_change_thr=%lu", 
             ulp_low_thr, ulp_high_thr, ulp_rapid_change_thr);
#if (C_LOG_LEVEL < 3)
    debug_ulp_status();
#endif
    return ESP_OK;
error:
    return ESP_FAIL;
}

void start_ulp_program(void)
{
    FUNC_ENTRY(TAG);
    
    /* Validate button GPIO configuration consistency */
    #ifdef CONFIG_LOGGER_BUTTON_ENABLED
    if (CONFIG_ULP_BUTTON_GPIO != CONFIG_LOGGER_BUTTON_GPIO_0) {
        WLOG(TAG, "WARNING: ULP button GPIO (%d) differs from main button GPIO (%d)", 
             CONFIG_ULP_BUTTON_GPIO, CONFIG_LOGGER_BUTTON_GPIO_0);
        WLOG(TAG, "Both should use the same GPIO for consistent button monitoring");
    }
    #endif
    
    if(adc_lock(1000)) {
        adc_unlock();
    }
    const bool preserve_history = ((ulp_last_wake_reason & UINT16_MAX) != 0);

    if (!preserve_history) {
        ulp_last_result = 0;
        for (int i = 0; i < RESULT_SLOTS; i++) {
            ulp_prev_result[i] = 0;
        }
        ulp_prev_result_idx = 0;
    }
    ulp_cum_change = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        ELOG(TAG, "Failed to start ULP program: %s\n", esp_err_to_name(err));
        return;
    }
}

void adc_ulp_clear_last_wake_reason(void)
{
    ulp_last_wake_reason = 0;
}

uint32_t adc_ulp_get_last_wake_reason(void)
{
    return ulp_last_wake_reason & UINT16_MAX;
}

/**
 * Check if ULP woke up due to button long press
 */
bool ulp_button_long_press_detected(void)
{
    return (ulp_wake_source & ULP_WAKE_SOURCE_BUTTON) && 
           (ulp_button_wake_reason == ULP_BUTTON_WAKE_LONG_PRESS);
}

/**
 * Check if ULP woke up due to ADC threshold
 */
bool ulp_adc_threshold_triggered(void)
{
    return (ulp_wake_source & ULP_WAKE_SOURCE_ADC) && 
           (ulp_adc_wake_reason != 0);
}

/**
 * Get the specific ADC wake reason (low, high, or rapid change)
 */
uint32_t ulp_get_adc_wake_reason(void)
{
    return ulp_adc_wake_reason;
}

/**
 * Clear all wake sources and reasons
 */
void ulp_clear_wake_sources(void)
{
    ulp_wake_source = 0;
    ulp_button_wake_reason = 0;
    ulp_adc_wake_reason = 0;
    ulp_last_wake_reason = 0;
}

/**
 * Diagnostic function to debug ULP status
 */
void debug_ulp_status(void) {
#if (C_LOG_LEVEL < 3)
    printf("=== ULP Diagnostic Status ===\n");
    printf("ULP Low Threshold: %lu\n", ulp_low_thr);
    printf("ULP High Threshold: %lu\n", ulp_high_thr);
    printf("ULP Rapid Change Threshold: %lu\n", ulp_rapid_change_thr);
    // printf("ULP Sample Counter: %lu", ulp_adc_counter & UINT16_MAX);
    printf("ULP Last Result: %lu\n", ulp_last_result & UINT16_MAX);
    printf("ULP Cumulative Change: %lu\n", ulp_cum_change & UINT16_MAX);
    printf("ULP Last Result: %lu\n", ulp_last_result & UINT16_MAX);
    printf("ULP Prev Result Index: %lu\n", ulp_prev_result_idx & UINT16_MAX);
    printf("ULP Prev Result: [");
    for (int i = 0; i < RESULT_SLOTS; i++) printf("%lu%s", ulp_prev_result[i] & UINT16_MAX, i < (RESULT_SLOTS - 1) ? ", " : "]\n");
    printf("=== End ULP Diagnostic ===\n");
#endif
}

/**
 * Get battery state using ULP variables when waking from ULP sleep
 * This function analyzes ULP ADC results to determine what triggered the wakeup
 * ULP ADC range: ~1800-2700 (vs main ADC: ~3300-4400), so we work with raw values
 */
adc_battery_state_t get_battery_state_from_ulp(void) {
    FUNC_ENTRY(TAG);
    // Get ULP variables (raw ADC values, not voltage)
    const uint32_t current_result = ulp_last_result & UINT16_MAX;   // Current ADC reading
    uint32_t prev_result = 0; // Previous reading
    for (uint8_t i = 0; i < RESULT_SLOTS; i++) {
        prev_result += ulp_prev_result[i] & UINT16_MAX;
    }
    prev_result /= RESULT_SLOTS; // Average previous readings
    const int32_t change = (int32_t)current_result - (int32_t)prev_result;
    const uint16_t low_thr = ADC_LOW_TRESHOLD;             // Low threshold from config
    const uint16_t high_thr = ADC_HIGH_TRESHOLD;           // High threshold from config
    const uint16_t rapid_thr = ADC_RAPID_CHANGE_TRESHOLD;  // Rapid change threshold

    DLOG(TAG, "ULP state analysis: current=%lu, change=%ld, low_thr=%hu, high_thr=%hu, rapid_thr=%hu",
         current_result, change, low_thr, high_thr, rapid_thr);
    
    // Check for low battery condition (primary ULP function)
    if (current_result <= low_thr) {
        WLOG(TAG, "ULP: detected low battery.");
        return ADC_BATTERY_LOW;
    }
    
    // Check for high battery condition (battery full detection)
    if (current_result >= high_thr) {
        ILOG(TAG, "ULP: detected high battery.");
        return ADC_BATTERY_HIGH;
    }
    
    // Check for rapid voltage changes (charging events detection)
    if (change != 0) {  // Only if we have a valid previous reading
        if ((change < -rapid_thr) || (change > rapid_thr)) {
            // Determine direction of change for charging detection
            if (change > 0) {  // Significant positive change
                ILOG(TAG, "ULP: Rapid increase suggests charging started.");
                return ADC_BATTERY_CHARGING_STARTED;
            } else if (change < 0) {  // Significant negative change
                ILOG(TAG, "ULP: Rapid decrease suggests charging stopped.");
                return ADC_BATTERY_CHARGING_STOPPED;
            }
        }
    }
    
    // Default to normal state - ULP woke us but no specific condition detected
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "ULP battery state: normal.");
#endif
    return ADC_BATTERY_NORMAL;
}

#endif /* CONFIG_ULP_COPROC_ENABLED */