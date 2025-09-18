#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_ENABLED)
#include "common_log.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#if defined(CONFIG_ULP_COPROC_ENABLED)
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "hal/rtc_hal.h"
#endif

#include <esp_idf_version.h>

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#include "esp_adc/adc_continuous.h"
#endif
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "adc.h"
#include "adc_events.h"
#include "driver/gpio.h"

/* ULP-specific includes */
#if defined(CONFIG_ULP_COPROC_ENABLED)
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_adc.h"
#include "ulp_common.h"
#include "ulp_common_defs.h"

/* ULP binary references */
extern const uint8_t ulp_logger_adc_bin_start[] asm("_binary_ulp_logger_adc_bin_start");
extern const uint8_t ulp_logger_adc_bin_end[]   asm("_binary_ulp_logger_adc_bin_end");

/* ULP variable references */
extern uint32_t ulp_entry;

uint32_t __attribute__((section(".rtc.data"))) ulp_low_thr = 0;
uint32_t __attribute__((section(".rtc.data"))) ulp_high_thr = 0;
uint32_t __attribute__((section(".rtc.data"))) ulp_sample_counter = 0;
uint32_t __attribute__((section(".rtc.data"))) ulp_last_result = 0;

/* ULP state tracking for battery state detection */
static uint32_t last_ulp_reading = 0;
static adc_battery_state_t last_battery_state = ADC_BATTERY_NORMAL;

/* Access to RTC slow memory - ESP-IDF 5.x uses direct addressing */
#if !defined(RTC_SLOW_MEM)
#define RTC_SLOW_MEM ((uint32_t*)0x50000000)  
#endif
/* ULP battery state enumeration and variables */
/* adc_battery_state_t enum is now defined in adc.h */

#endif

/* Regular ADC battery state tracking (when ULP is disabled) */
static uint32_t last_adc_reading_mv = 0;
static adc_battery_state_t last_adc_battery_state = ADC_BATTERY_NORMAL;

/* Enhanced voltage filtering for LilyGO T5 ADC stability */
static float voltage_history[VOLTAGE_HISTORY_SIZE] = {0};
static uint8_t voltage_history_index = 0;
static uint32_t last_voltage_update_ms = 0;
static float filtered_voltage = 0.0f;  // Exponential moving average
static bool filter_initialized = false;

ESP_EVENT_DEFINE_BASE(ADC_EVENT);
#if (C_LOG_LEVEL < 3)
static const char * _adc_event_strings[] = { ADC_EVENT_LIST(STRINGIFY) };
const char * adc_event_strings(int id) {
    return _adc_event_strings[id];
}
#else
const char * adc_event_strings(int id) {return "ADC_EVENT";}
#endif

static const char *TAG = "adc";

#define V_GRAPH_LIPO_LEN 21
#define ADJ_LENGTH 24

typedef struct adc_context_s {
    uint8_t on_ac;
    uint32_t adc_raw;
    uint32_t adc_voltage;
    uint8_t do_calibration;
    adc_cali_handle_t adc1_cali_handle;
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    uint32_t running_sum;
    uint32_t running_avg;
    uint32_t m_avg[3];
#endif
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_handle_t adc1_handle;
    esp_timer_handle_t adc_periodic_timer;
    uint32_t result[RESULT_SIZE];
    int32_t result_index;
    SemaphoreHandle_t xMutex;
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_continuous_handle_t adc1_handle;
    TaskHandle_t adc_task_handle;
    uint32_t ret_num;
    uint8_t result[READ_LEN];
    uint8_t task_is_running;
#endif
} adc_context_t;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
#define CTX_PART .adc_periodic_timer = NULL, \
    .result_index = -1, \
    .xMutex = NULL,
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
#define CTX_PART .adc_task_handle = NULL, \
    .ret_num = 0, \
    .result = {0}, \
    .task_is_running = 1,
#endif
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
#define AC_DET_PART .on_ac = 0, \
    .running_sum = 0, \
    .running_avg = 0, \
    .m_avg = {0},
#else
#define AC_DET_PART
#endif
#define ADC_CONTEXT_DEFAULT { \
    .on_ac = 0, \
    .adc_raw = 0, \
    .adc_voltage = 0, \
    .do_calibration = 0, \
    .adc1_cali_handle = NULL, \
    .adc1_handle = NULL, \
    .result = {0}, \
    AC_DET_PART \
    CTX_PART \
}
static adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;

static const uint16_t v_graph_lipo[V_GRAPH_LIPO_LEN] = {
    33000,  // 0
    36100,  // 5
    36900,  // 10
    37100,  // 15
    37300,  // 20
    37500,  // 25
    37700,  // 30
    37900,  // 35
    38000,  // 40
    38200,  // 45
    38400,  // 50
    38500,  // 55
    38700,  // 60
    39100,  // 65
    39500,  // 70
    39800,  // 75
    40200,  // 80
    40800,  // 85
    41100,  // 90
    41500,  // 95
    42000,  // 100
};

/* ULP-related functions */
#if defined(CONFIG_ULP_COPROC_ENABLED)

void configure_adc_pad(void)
{
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
    
    ESP_LOGI(TAG, "ADC pad GPIO%d (channel %d) configured for ULP", adc_gpio, _ADC_CHANNEL_0);
}

/**
 * Initialize the ULP program for battery monitoring
 */
esp_err_t init_ulp_program(void) {
    esp_err_t err = ESP_OK;
    
    ILOG(TAG, "Initializing ULP program...");
    
    // IMPORTANT: Initialize ULP ADC FIRST before loading the program
    // This ensures ADC hardware is ready for ULP use
    ILOG(TAG, "Initializing ULP ADC: Unit=%d, Channel=%d, Attenuation=%d", _ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN);
    
    // Critical check: verify ULP assembly and C code are using the same channel
    #include "adc_config.h"
    ILOG(TAG, "ULP Assembly Channel: %d, C Code Channel: %d", ULP_ADC_CHANNEL, (int)_ADC_CHANNEL_0);
    if (ULP_ADC_CHANNEL != (int)_ADC_CHANNEL_0) {
        ELOG(TAG, "CRITICAL: ULP assembly channel (%d) != C code channel (%d)!", ULP_ADC_CHANNEL, (int)_ADC_CHANNEL_0);
        return ESP_ERR_INVALID_ARG;
    }
    
    ulp_adc_cfg_t adc_cfg = {
        .adc_n = _ADC_UNIT_0,     // Use same unit as regular ADC
        .channel = _ADC_CHANNEL_0, // Use same channel as regular ADC  
        .atten = _ADC_ATTEN,      // Use same attenuation as regular ADC
        .ulp_mode = ADC_ULP_MODE_FSM, // Explicitly specify FSM mode for ESP32 (not RISC-V)
    };
    err = ulp_adc_init(&adc_cfg);
    if (err != ESP_OK) {
        ELOG(TAG, "Failed to initialize ULP ADC: %s", esp_err_to_name(err));
        return err;
    }
    ILOG(TAG, "ULP ADC initialized (Unit %d, Channel %d, Attenuation %d, FSM mode)", 
         _ADC_UNIT_0 + 1, _ADC_CHANNEL_0, _ADC_ATTEN);

    // Configure the ADC GPIO pin for ULP use
    configure_adc_pad();

    // First, let's verify the ULP binary was loaded correctly
    const size_t ulp_prog_size = ulp_logger_adc_bin_end - ulp_logger_adc_bin_start;
    
    ILOG(TAG, "ULP binary size: %zu bytes", ulp_prog_size);
    ILOG(TAG, "ULP entry point address: 0x%08lx", (unsigned long)&ulp_entry);
    err = ulp_load_binary(0, ulp_logger_adc_bin_start, ulp_prog_size / sizeof(uint32_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load ULP program: %s", esp_err_to_name(err));
        return err;
    }
    // Load ULP program into RTC memory

    // Set ULP wakeup thresholds based on ADC raw values (not mV)
    // Convert voltage thresholds to approximate ADC raw values
    // Assuming ~3.3V reference and 12-bit ADC: 1mV ≈ 1.24 ADC units
    uint32_t low_threshold_raw = (ULP_BATTERY_LOW_THRESHOLD * 1024) / 3300;   // ~1100 for 3.6V
    uint32_t high_threshold_raw = (ULP_BATTERY_HIGH_THRESHOLD * 1024) / 3300; // ~1270 for 4.1V
    
    ILOG(TAG, "Setting ULP thresholds: low=%lu (%.1fV), high=%lu (%.1fV)", 
         low_threshold_raw, (float)ULP_BATTERY_LOW_THRESHOLD/1000.0f,
         high_threshold_raw, (float)ULP_BATTERY_HIGH_THRESHOLD/1000.0f);
         
    // Set threshold values in ULP variables
    ulp_low_thr = low_threshold_raw;
    ulp_high_thr = high_threshold_raw;
    
    // Note: ulp_last_result and ulp_sample_counter are already initialized to 0 in adc.S .bss section
    
    // Set initial values in RTC memory for our application
    RTC_SLOW_MEM[ULP_ADC_READING_ADDR] = 0;
    RTC_SLOW_MEM[ULP_BATTERY_STATE_ADDR] = ADC_BATTERY_NORMAL;
    RTC_SLOW_MEM[ULP_WAKE_FLAG_ADDR] = 0;
    
    // ESP-IDF 5.x: Configure slow clock FIRST - this is critical
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for clock to stabilize
    
    // Configure ULP wakeup period BEFORE starting the program
    ulp_set_wakeup_period(0, ULP_READ_PERIOD_MS * 1000); // Convert ms to microseconds
    vTaskDelay(pdMS_TO_TICKS(10)); // Brief pause
    
    // Start the ULP program - use offset from RTC memory start like in ESP-IDF example
    uint32_t ulp_entry_offset = (uint32_t)(&ulp_entry - RTC_SLOW_MEM);
    ILOG(TAG, "ULP entry offset: %lu words", ulp_entry_offset);
    err = ulp_run(ulp_entry_offset);
    if (err != ESP_OK) {
        ELOG(TAG, "Failed to start ULP program: %s", esp_err_to_name(err));
        return err;
    }
    
    // Force enable the ULP timer - use proper API for ESP-IDF 5.x
    ulp_timer_resume();
    
    ILOG(TAG, "ULP program initialized and started successfully");
    
    // Give ULP some time to take its first measurement
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check if ULP is working by reading the sample counter after a short delay
    uint32_t sample_count = ulp_sample_counter;
    ILOG(TAG, "ULP sample counter after start: %lu", sample_count);
    
    // Test manual ULP execution
    ILOG(TAG, "Testing manual ULP execution...");
    manual_trigger_ulp_measurement();
    
    return ESP_OK;
}

/**
 * Get current battery state from ULP
 */
adc_battery_state_t get_ulp_battery_state(void) {
    return (adc_battery_state_t)RTC_SLOW_MEM[ULP_BATTERY_STATE_ADDR];
}

/**
 * Get raw ULP ADC reading
 */
uint32_t get_ulp_adc_reading(void) {
    uint32_t raw_reading = RTC_SLOW_MEM[ULP_ADC_READING_ADDR];
    uint32_t ulp_result = ulp_last_result;
    uint32_t sample_count = ulp_sample_counter;
    
    // Debug: Check both our RTC memory location and ULP's last_result
    if (raw_reading > 0) {
        DLOG(TAG, "ULP RTC_SLOW_MEM reading: %lu", raw_reading);
    }
    
    if (ulp_result > 0) {
        DLOG(TAG, "ULP last_result: %lu, samples: %lu", ulp_result, sample_count);
        // Prefer the ULP's own result storage
        return ulp_result;
    }
    
    if (raw_reading > 0) {
        return raw_reading;
    }
    
    // No valid reading from either location
    DLOG(TAG, "No valid ULP reading available (RTC_MEM=%lu, ulp_result=%lu, samples=%lu)", 
         raw_reading, ulp_result, sample_count);
    return 0;
}

/**
 * Get calibrated ULP ADC reading in millivolts
 * Applies proper ADC calibration without averaging (ULP is already stable)
 * Includes smart validation for shared pin scenarios (e.g., LilyGO boards)
 */
uint32_t get_ulp_adc_calibrated(void) {
    uint32_t raw_reading = get_ulp_adc_reading();
    if (raw_reading == 0) {
        return 0; // No ULP reading available
    }
    
    // Validate raw reading range (12-bit ADC)
    if (raw_reading > 4095) {  // ESP32-S3 ADC is 12-bit max
        WLOG(TAG, "ULP ADC reading %lu exceeds 12-bit range - possible pin conflict", raw_reading);
        return 0;
    }
    if (raw_reading < 50) {    // Extremely low readings suggest pin issues
        DLOG(TAG, "ULP ADC reading %lu extremely low - possible pin conflict", raw_reading);
        return 0;
    }
    
    // Use unified calibration logic
    uint32_t calibrated_voltage = calibrate_adc_raw(raw_reading);
    if (calibrated_voltage == 0) {
        WLOG(TAG, "ULP ADC calibration failed for raw reading %lu", raw_reading);
        return 0;
    }
    
    // Use unified validation logic
    if (!validate_voltage_reading(calibrated_voltage, "ULP")) {
        return 0;
    }
    
    DLOG(TAG, "ULP ADC: raw=%lu, calibrated=%lu mV", raw_reading, calibrated_voltage);
    return calibrated_voltage;
}

/**
 * Common battery state event posting function
 * Handles posting ESP events for battery state changes
 */
static void post_battery_state_event(adc_battery_state_t state, float voltage_mv, const char* source);

/**
 * Common voltage validation function for both ULP and regular ADC
 * Validates voltage readings against board-specific thresholds
 * @param voltage_mv Voltage in millivolts
 * @param source_name Source description for logging ("ULP" or "ADC")
 * @return true if voltage is valid, false if likely pin conflict or unrealistic
 */
bool validate_voltage_reading(uint32_t voltage_mv, const char* source_name) {
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    // T-Display S3: Can show > 4.3V during charging, > 5.5V suggests pin conflict
    if (voltage_mv > 5500) {  // Above realistic USB charging voltage
        WLOG(TAG, "%s voltage %lu mV exceeds USB charging range - pin conflict detected", source_name, voltage_mv);
        return false;
    }
    if (voltage_mv < 2000) {   // Below realistic system operating voltage
        DLOG(TAG, "%s voltage %lu mV below realistic operating range", source_name, voltage_mv);
        return false;
    }
    
    // Log charging detection for T-Display S3
    if (voltage_mv > 4300) {
        DLOG(TAG, "%s charging detected: %lu mV (USB connected)", source_name, voltage_mv);
    }
#else
    // T5 and other boards: More conservative voltage range
    if (voltage_mv > 4500) {  // Above realistic max with battery connected
        WLOG(TAG, "%s voltage %lu mV exceeds expected range - pin conflict detected", source_name, voltage_mv);
        return false;
    }
    if (voltage_mv < 2000) {   // Below realistic operating voltage
        DLOG(TAG, "%s voltage %lu mV below realistic operating range", source_name, voltage_mv);
        return false;
    }
#endif
    return true;
}

/**
 * Check if ULP caused wake-up and handle battery events
 * Uses the same improved battery state logic as regular ADC for consistency
 */
void handle_ulp_wakeup(void) {
    uint32_t wake_flag = RTC_SLOW_MEM[ULP_WAKE_FLAG_ADDR];
    uint32_t current_reading_mv = get_ulp_adc_calibrated();
    
    if (wake_flag && current_reading_mv > 0) {
        ILOG(TAG, "ULP wake-up detected, voltage: %lu mV", current_reading_mv);
        
        // Clear wake flag
        RTC_SLOW_MEM[ULP_WAKE_FLAG_ADDR] = 0;
        
        // Use the same improved battery state detection as regular ADC
        adc_battery_state_t current_state = get_adc_battery_state(current_reading_mv);
        
        // Only post events on state changes
        if (current_state != last_battery_state) {
            DLOG(TAG, "ULP battery state changed: %d -> %d (%lu mV)", 
                 last_battery_state, current_state, current_reading_mv);
            
            post_battery_state_event(current_state, (float)current_reading_mv, "ULP");
            
            last_battery_state = current_state;
        }
        
        last_ulp_reading = current_reading_mv;
    }
}

/**
 * Configure ULP as wake source
 */
esp_err_t configure_ulp_wakeup(void) {
    // Enable ULP wakeup
    esp_err_t err = esp_sleep_enable_ulp_wakeup();
    if (err != ESP_OK) {
        ELOG(TAG, "Failed to enable ULP wakeup: %s", esp_err_to_name(err));
        return err;
    }
    
    ILOG(TAG, "ULP wakeup configured successfully");
    return ESP_OK;
}

/**
 * Diagnostic function to debug ULP status
 */
void debug_ulp_status(void) {
    ILOG(TAG, "=== ULP Diagnostic Status ===");
    ILOG(TAG, "ULP ADC Channel: %d", ULP_ADC_CHANNEL);
    ILOG(TAG, "ULP Reading Period: %d ms", ULP_READ_PERIOD_MS);
    ILOG(TAG, "ULP Low Threshold: %lu", ulp_low_thr);
    ILOG(TAG, "ULP High Threshold: %lu", ulp_high_thr);
    ILOG(TAG, "ULP Sample Counter: %lu", ulp_sample_counter);
    ILOG(TAG, "ULP Last Result: %lu", ulp_last_result);
    ILOG(TAG, "RTC_SLOW_MEM[0] (ADC_READING): %lu", RTC_SLOW_MEM[ULP_ADC_READING_ADDR]);
    ILOG(TAG, "RTC_SLOW_MEM[1] (BATTERY_STATE): %lu", RTC_SLOW_MEM[ULP_BATTERY_STATE_ADDR]);
    ILOG(TAG, "RTC_SLOW_MEM[2] (WAKE_FLAG): %lu", RTC_SLOW_MEM[ULP_WAKE_FLAG_ADDR]);
    ILOG(TAG, "=== End ULP Diagnostic ===");
}

/**
 * Manually trigger ULP measurement for testing
 * This forces the ULP to run one measurement cycle
 */
esp_err_t manual_trigger_ulp_measurement(void) {
    ILOG(TAG, "=== Manual ULP Trigger Test ===");
    
    // Store current values for comparison
    uint32_t sample_count_before = ulp_sample_counter;
    uint32_t last_result_before = ulp_last_result;
    
    ILOG(TAG, "Before trigger: samples=%lu, result=%lu", sample_count_before, last_result_before);
    
    // Method 1: Try to manually run ULP program once
    esp_err_t err = ulp_run((uint32_t)(&ulp_entry - RTC_SLOW_MEM));  // Use offset like ESP-IDF example
    if (err != ESP_OK) {
        ELOG(TAG, "Manual ULP run failed: %s", esp_err_to_name(err));
    } else {
        ILOG(TAG, "Manual ULP run initiated");
    }
    
    // Give ULP time to execute
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Check if anything changed
    uint32_t sample_count_after = ulp_sample_counter;
    uint32_t last_result_after = ulp_last_result;
    
    ILOG(TAG, "After trigger: samples=%lu (+%ld), result=%lu", 
         sample_count_after, (int32_t)(sample_count_after - sample_count_before), last_result_after);
    
    if (sample_count_after > sample_count_before) {
        ILOG(TAG, "✅ ULP executed! Sample counter increased");
        if (last_result_after > 0) {
            ILOG(TAG, "✅ ULP measurement successful: %lu", last_result_after);
            // Copy ULP result to our application memory
            RTC_SLOW_MEM[ULP_ADC_READING_ADDR] = last_result_after;
            return ESP_OK;
        } else {
            WLOG(TAG, "⚠️ ULP executed but no valid result");
            return ESP_FAIL;
        }
    } else {
        WLOG(TAG, "❌ ULP did not execute - sample counter unchanged");
        return ESP_FAIL;
    }
}

/**
 * Test ULP functionality step by step
 */
void test_ulp_functionality(void) {
    ILOG(TAG, "=== ULP Functionality Test ===");
    
    // Test 1: Check if ULP variables are accessible
    ILOG(TAG, "Test 1: ULP Variable Access");
    ulp_sample_counter = 42;  // Test write
    uint32_t test_read = ulp_sample_counter;  // Test read
    if (test_read == 42) {
        ILOG(TAG, "✅ ULP variable access working (wrote 42, read %lu)", test_read);
        ulp_sample_counter = 0;  // Reset
    } else {
        ELOG(TAG, "❌ ULP variable access failed (wrote 42, read %lu)", test_read);
    }
    
    // Test 2: Check threshold values
    ILOG(TAG, "Test 2: ULP Threshold Values");
    ILOG(TAG, "ulp_low_thr = %lu (expected ~1055)", ulp_low_thr);
    ILOG(TAG, "ulp_high_thr = %lu (expected ~1272)", ulp_high_thr);
    
    // Test 3: Manual measurement trigger
    ILOG(TAG, "Test 3: Manual ULP Measurement");
    manual_trigger_ulp_measurement();
    
    // Test 4: Compare with direct ADC reading
    ILOG(TAG, "Test 4: Direct ADC Comparison");
    // Get current raw ADC value from context instead of calling internal function
    ILOG(TAG, "Current ADC context raw: %lu", adc_ctx.adc_raw);
    ILOG(TAG, "ULP should read similar values on channel %d", ULP_ADC_CHANNEL);
    
    ILOG(TAG, "=== End ULP Functionality Test ===");
}

#endif /* CONFIG_ULP_COPROC_ENABLED */

/**
 * Common calibration function for both ULP and regular ADC readings
 * Applies hardware calibration if available, falls back to voltage conversion
 * @param raw_adc Raw ADC reading value
 * @return Calibrated voltage in millivolts, or 0 on error
 */
uint32_t calibrate_adc_raw(uint32_t raw_adc) {
    if (raw_adc == 0) {
        return 0; // Invalid reading
    }
    
    uint32_t calibrated_voltage = 0;
    
    // Apply hardware calibration if available
    if (adc_ctx.do_calibration && adc_ctx.adc1_cali_handle) {
        int temp_voltage; // ESP-IDF calibration API expects int*
        if (adc_cali_raw_to_voltage(adc_ctx.adc1_cali_handle, raw_adc, &temp_voltage) != ESP_OK) {
            // Fallback to voltage conversion without calibration
            calibrated_voltage = VOLTAGE_CONV((float)raw_adc);
        } else {
            calibrated_voltage = (uint32_t)temp_voltage;
        }
    } else {
        // No calibration handle available, use voltage conversion
        calibrated_voltage = VOLTAGE_CONV((float)raw_adc);
    }
    
    return calibrated_voltage;
}

/**
 * Helper function to get the most recent ADC reading as voltage
 * Consolidates duplicated voltage conversion logic
 */
static inline float get_recent_voltage_reading(void) {
    return VOLTAGE_U32_TO_V(((float)adc_ctx.result[adc_ctx.result_index % RESULT_SIZE]));
}

/**
 * Helper function to get current buffer index safely
 * Consolidates duplicated buffer indexing logic
 */
static inline uint8_t get_current_buffer_index(void) {
    return adc_ctx.result_index % RESULT_SIZE;
}

/**
 * Helper function to get previous buffer index safely
 * Consolidates duplicated buffer indexing logic
 */
static inline uint8_t get_previous_buffer_index(void) {
    return (adc_ctx.result_index - 1) % RESULT_SIZE;
}

/**
 * Helper function for voltage validation and clamping
 * Consolidates duplicated voltage validation logic for different board types
 */
float validate_and_clamp_voltage(float voltage, bool is_display_s3) {
    const float FALLBACK_VOLTAGE = FALLBACK_VOLTAGE_LILYGO;
    
    if (is_display_s3) {
        // T-Display S3 validation
        if (voltage > USB_VOLTAGE_MAX) {  // Above USB charging range suggests real pin conflict
            WLOG(TAG, "T-Display S3 continuous ADC voltage %f exceeds charging range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage < BATTERY_VOLTAGE_MIN) {  // Below realistic operating range
            WLOG(TAG, "T-Display S3 continuous ADC voltage %f below realistic range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage > CHARGING_VOLTAGE_THRESHOLD) {
            // Charging detected - this is normal behavior for T-Display S3
            DLOG(TAG, "T-Display S3 charging detected: %f V", voltage);
        }
    } else {
        // T5 and other boards: More conservative validation
        if (voltage > BATTERY_VOLTAGE_MAX_T5) {  // Above realistic range for T5 with battery
            WLOG(TAG, "Continuous ADC voltage %f exceeds T5 expected range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        } else if (voltage < BATTERY_VOLTAGE_MIN) {  // Below realistic operating range
            WLOG(TAG, "Continuous ADC voltage %f below realistic range - possible pin conflict", voltage);
            return FALLBACK_VOLTAGE;
        }
    }
    
    return voltage; // No clamping needed
}

/**
 * Common battery state event posting function
 * Handles posting ESP events for battery state changes
 */
static void post_battery_state_event(adc_battery_state_t state, float voltage_mv, const char* source) {
    switch (state) {
        case ADC_BATTERY_LOW:
            WLOG(TAG, "%s detected low battery: %.1f mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_LOW, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_HIGH:
            ILOG(TAG, "%s detected high battery: %.1f mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_HIGH, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STARTED:
            ILOG(TAG, "%s detected charging started: %.1f mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STARTED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
            ILOG(TAG, "%s detected charging stopped: %.1f mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGE_STOPPED, NULL, 0, pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            ELOG(TAG, "%s detected critical low battery: %.1f mV", source, voltage_mv);
            esp_event_post(ADC_EVENT, ADC_EVENT_BATTERY_CRITICAL, NULL, 0, pdMS_TO_TICKS(100));
            break;
        default:
            // Normal state - no event needed
            break;
    }
}

/**
 * Get current battery state for both ULP and regular ADC modes
 * Uses hysteresis for battery levels but sensitive charging detection for charger events
 * Battery-only operation is more stable, so hysteresis prevents false low/high transitions
 * Charging detection uses trend analysis to catch real charger connect/disconnect events
 */
adc_battery_state_t get_battery_state(uint32_t voltage_mv) {
    // Use same thresholds as ULP for consistency, but add hysteresis for T5 charging stability
    const uint32_t critical_low_mv = 3200;
    const uint32_t low_mv = 3400;
    const uint32_t low_hysteresis_mv = 3500;  // Higher threshold to exit low state
    const uint32_t high_mv = 4100;
    const uint32_t high_hysteresis_mv = 4000; // Lower threshold to exit high state
    
    // Detect charging by voltage increase rate (same logic as ULP)
    static uint32_t last_voltage_mv = 0;
    static uint32_t last_check_ms = 0;
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    adc_battery_state_t state = ADC_BATTERY_NORMAL;
    
    // Get current state for hysteresis logic
    static adc_battery_state_t current_state = ADC_BATTERY_NORMAL;
    
    // Check for critical low battery first (no hysteresis needed for safety)
    if (voltage_mv < critical_low_mv) {
        state = ADC_BATTERY_CRITICAL_LOW;
    }
    // Check for low battery with hysteresis
    else if (current_state == ADC_BATTERY_LOW) {
        // Already in low state - use higher threshold to exit
        if (voltage_mv < low_hysteresis_mv) {
            state = ADC_BATTERY_LOW;
        } else {
            state = ADC_BATTERY_NORMAL;
        }
    } else {
        // Not in low state - use lower threshold to enter
        if (voltage_mv < low_mv) {
            state = ADC_BATTERY_LOW;
        }
    }
    
    // Check for high battery with hysteresis
    if (current_state == ADC_BATTERY_HIGH) {
        // Already in high state - use lower threshold to exit
        if (voltage_mv >= high_hysteresis_mv) {
            state = ADC_BATTERY_HIGH;
        } else if (state == ADC_BATTERY_NORMAL) {
            // Keep high state if not overridden by low state
            state = ADC_BATTERY_HIGH;
        }
    } else {
        // Not in high state - use higher threshold to enter
        if (voltage_mv >= high_mv) {
            state = ADC_BATTERY_HIGH;
        }
    }
    
    // Detect charging state changes based on voltage rate - with hysteresis to prevent oscillation
    // Use moderate time window and higher thresholds to avoid false positives from ADC noise
    static uint32_t charge_detect_samples[3] = {0}; // Keep last 3 readings for trend analysis
    static uint8_t charge_sample_index = 0;
    static uint32_t last_charge_check_ms = 0;
    static uint32_t last_charge_state_change_ms = 0;
    static adc_battery_state_t last_charge_state = ADC_BATTERY_NORMAL;

    // Update charge detection samples every reading
    charge_detect_samples[charge_sample_index] = voltage_mv;
    charge_sample_index = (charge_sample_index + 1) % 3;

    // Check charging trends every 2 seconds (moderate frequency for ~5 second averaging)
    if (last_voltage_mv > 0 && (current_ms - last_charge_check_ms) > 2000) {
        // Use trend over last 3 samples (~5 seconds) for balanced detection
        uint32_t oldest_sample = charge_detect_samples[(charge_sample_index + 2) % 3];
        uint32_t newest_sample = charge_detect_samples[charge_sample_index];

        // Calculate trend: compare oldest vs newest over ~5 seconds
        int32_t trend_diff = (int32_t)newest_sample - (int32_t)oldest_sample;

        // Require minimum time between charging state changes (30 seconds) to prevent oscillation
        bool can_change_state = (current_ms - last_charge_state_change_ms) > 30000;

        // Higher thresholds with hysteresis to prevent false charging detection
        if (trend_diff > 150) {  // > 150mV increase over 5 seconds indicates charging started
            if (last_charge_state != ADC_BATTERY_CHARGING_STARTED && can_change_state) {
                state = ADC_BATTERY_CHARGING_STARTED;
                last_charge_state = ADC_BATTERY_CHARGING_STARTED;
                last_charge_state_change_ms = current_ms;
                DLOG(TAG, "ADC detected charging started: %lu mV (+%ld mV trend over 5s)", voltage_mv, trend_diff);
            }
        }
        else if (trend_diff < -100) {  // > 100mV decrease over 5 seconds indicates charging stopped
            if (last_charge_state != ADC_BATTERY_CHARGING_STOPPED && can_change_state) {
                state = ADC_BATTERY_CHARGING_STOPPED;
                last_charge_state = ADC_BATTERY_CHARGING_STOPPED;
                last_charge_state_change_ms = current_ms;
                DLOG(TAG, "ADC detected charging stopped: %lu mV (%ld mV trend over 5s)", voltage_mv, trend_diff);
            }
        }

        last_charge_check_ms = current_ms;
    }

    // Immediate detection for large voltage jumps (charger connect/disconnect)
    // Check for sudden voltage increase that indicates charger connection
    if (last_voltage_mv > 0) {
        int32_t voltage_diff = (int32_t)voltage_mv - (int32_t)last_voltage_mv;
        
        if (voltage_diff > 400) {
            // Voltage jumped by more than 400mV, indicating charger connection
            if (last_charge_state != ADC_BATTERY_CHARGING_STARTED &&
                (current_ms - last_charge_state_change_ms) > 5000) {  // 5 second minimum
                state = ADC_BATTERY_CHARGING_STARTED;
                last_charge_state = ADC_BATTERY_CHARGING_STARTED;
                last_charge_state_change_ms = current_ms;
                DLOG(TAG, "ADC detected charging started: %lu mV (sudden jump +%ld mV from %lu mV)", 
                     voltage_mv, voltage_diff, last_voltage_mv);
            }
        }
        else if (voltage_diff < -300) {
            // Voltage dropped by more than 300mV, indicating charger disconnection
            if (last_charge_state != ADC_BATTERY_CHARGING_STOPPED &&
                (current_ms - last_charge_state_change_ms) > 5000) {  // 5 second minimum
                state = ADC_BATTERY_CHARGING_STOPPED;
                last_charge_state = ADC_BATTERY_CHARGING_STOPPED;
                last_charge_state_change_ms = current_ms;
                DLOG(TAG, "ADC detected charging stopped: %lu mV (sudden drop %ld mV from %lu mV)", 
                     voltage_mv, voltage_diff, last_voltage_mv);
            }
        }
    }
    
    last_voltage_mv = voltage_mv;
    current_state = state;  // Update current state for hysteresis
    return state;
}

/**
 * Handle battery state machine for regular ADC mode (when ULP is disabled)
 * Called from adc_update() to process new voltage readings
 */
void handle_adc_battery_state(float voltage_mv) {
    adc_battery_state_t current_state = get_battery_state((uint32_t)voltage_mv);
    
    // Only post events on state changes
    if (current_state != last_adc_battery_state) {
        DLOG(TAG, "ADC battery state changed: %d -> %d (%0.1f mV)", 
             last_adc_battery_state, current_state, voltage_mv);
        
        post_battery_state_event(current_state, voltage_mv, "ADC");
        
        last_adc_battery_state = current_state;
    }
    
    // Store the reading for rate-based charging detection
    last_adc_reading_mv = (uint32_t)voltage_mv;
}

/**
 * Backward compatibility wrapper for get_adc_battery_state
 */
adc_battery_state_t get_adc_battery_state(uint32_t voltage_mv) {
    return get_battery_state(voltage_mv);
}

/**
 * Enhanced voltage history update with exponential moving average filtering
 * Compensates for LilyGO T5 ADC instability and USB power noise
 */
static void update_voltage_history(float voltage) {
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Only update if enough time has passed (avoid excessive noise)
    if (current_ms - last_voltage_update_ms > 500) {  // 500ms interval for better responsiveness
        // Apply exponential moving average filter for noise reduction
        if (!filter_initialized) {
            // Initialize filter with first reading
            filtered_voltage = voltage;
            filter_initialized = true;
            DLOG(TAG, "Voltage filter initialized: %.3fV", filtered_voltage);
        } else {
            // Apply exponential moving average
            filtered_voltage = VOLTAGE_FILTER_ALPHA * voltage + (1.0f - VOLTAGE_FILTER_ALPHA) * filtered_voltage;
        }

        // Update circular buffer with filtered voltage
        voltage_history[voltage_history_index] = filtered_voltage;
        voltage_history_index = (voltage_history_index + 1) % VOLTAGE_HISTORY_SIZE;
        last_voltage_update_ms = current_ms;

        // Log filtered voltage for debugging (less verbose than raw history)
        static uint32_t last_log_ms = 0;
        if (current_ms - last_log_ms > 5000) {  // Log every 5 seconds
            float avg_voltage = 0.0f;
            uint8_t valid_readings = 0;
            for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
                if (voltage_history[i] > 0) {
                    avg_voltage += voltage_history[i];
                    valid_readings++;
                }
            }
            if (valid_readings > 0) {
                avg_voltage /= valid_readings;
                DLOG(TAG, "Filtered voltage: %.3fV (avg: %.3fV from %d readings)\n",
                     filtered_voltage, avg_voltage, valid_readings);
            }
            last_log_ms = current_ms;
        }
    }
}

/**
 * Detect charging based on filtered voltage increase rate (for T5 boards)
 * Uses exponential moving average to reduce false positives from ADC noise
 */
static bool detect_charging_by_rate(void) {
    // Need at least 3 readings to calculate rate reliably
    uint8_t valid_readings = 0;
    for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
        if (voltage_history[i] > 0) valid_readings++;
    }

    if (valid_readings < 3) {
        return false;
    }

    // Calculate voltage change rate over recent filtered history
    // Use the last 3 readings for rate calculation
    int idx1 = (voltage_history_index + VOLTAGE_HISTORY_SIZE - 1) % VOLTAGE_HISTORY_SIZE;
    int idx2 = (voltage_history_index + VOLTAGE_HISTORY_SIZE - 2) % VOLTAGE_HISTORY_SIZE;
    int idx3 = (voltage_history_index + VOLTAGE_HISTORY_SIZE - 3) % VOLTAGE_HISTORY_SIZE;

    if (voltage_history[idx1] == 0 || voltage_history[idx2] == 0 || voltage_history[idx3] == 0) {
        return false;
    }

    // Calculate average rate of change over last 3 readings
    float recent_change = voltage_history[idx1] - voltage_history[idx3];

    // Charging typically causes filtered voltage to rise by > 50mV over 3 readings (1.5 seconds)
    // Use lower threshold since we're using filtered values
    bool rapid_increase = recent_change > 0.05f;  // > 50mV increase

    if (rapid_increase) {
        DLOG(TAG, "Filtered rapid voltage increase detected: +%.3fV (charging started)", recent_change);
    }

    return rapid_increase;
}

/**
 * Validate ADC reading for shared pin conflicts (especially on LilyGO boards)
 * Returns true if reading seems valid, false if potentially corrupted by pin sharing
 * Note: T5 boards don't show > 4.3V during charging with battery connected
 */
bool validate_adc_reading(float voltage) {
    // Update voltage history for charging detection
    update_voltage_history(voltage);
    
    // Convert to millivolts and use unified validation
    uint32_t voltage_mv = (uint32_t)(voltage * 1000.0f);
    if (!validate_voltage_reading(voltage_mv, "ADC")) {
        return false;
    }
    
#if !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    // For T5: Detect charging by rate change, not absolute voltage
    if (detect_charging_by_rate()) {
        DLOG(TAG, "T5 charging detected by voltage increase rate: %.2fV", voltage);
    }
#endif
    return true;
}

/**
 * Get a safe battery voltage reading with conflict detection
 * Falls back to reasonable estimates if pin conflicts detected
 */
float get_safe_battery_voltage(void) {
    float voltage = volt_read();
    
    if (!validate_adc_reading(voltage)) {
        // Pin conflict detected, return a safe estimate
        WLOG(TAG, "Pin conflict detected, using fallback voltage estimate");
        
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
        // For LilyGO boards, use a conservative estimate
        voltage = FALLBACK_VOLTAGE_LILYGO;
#else
        // For other boards, use a more generic estimate
        voltage = FALLBACK_VOLTAGE_GENERIC;
#endif
    }
    
    return voltage;
}

/**
 * Detect if USB charging is active based on ADC reading and voltage change patterns
 * T-Display S3: voltage > 4.3V indicates USB charging
 * T5: charging detected by rapid voltage increase, not high voltage
 */
bool is_usb_charging(void) {
    float voltage = volt_read();
    
    // Validate reading first
    if (!validate_adc_reading(voltage)) {
        return false;  // Can't determine charging state with invalid reading
    }
    
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    // T-Display S3: Charging network pulls voltage above 4.3V when USB connected
    bool charging = voltage > CHARGING_VOLTAGE_THRESHOLD;
    
    if (charging) {
        DLOG(TAG, "T-Display S3 USB charging detected: %.2fV", voltage);
    }
    
    return charging;
#else
    // T5 and other boards: Detect charging by voltage increase rate and reasonable voltage
    bool charging_voltage_range = (voltage > CHARGING_RANGE_MIN && voltage <= CHARGING_VOLTAGE_THRESHOLD);  // Charging range for T5
    bool charging_rate = detect_charging_by_rate();
    
    bool charging = charging_voltage_range && charging_rate;
    
    if (charging) {
        DLOG(TAG, "T5 USB charging detected by rate change: %.2fV", voltage);
    }
    
    return charging;
#endif
}

/**
 * Get actual battery voltage (compensated for charging)
 * T-Display S3: When charging, ADC shows charging voltage, not actual battery voltage  
 * T5: ADC shows actual battery voltage even during charging
 */
float get_battery_voltage_compensated(void) {
    float voltage = volt_read();
    
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    if (voltage > 4.3f) {
        // During charging on T-Display S3, ADC shows charging network voltage, not battery voltage
        // Estimate actual battery voltage (typically 4.0-4.2V when charging)
        DLOG(TAG, "T-Display S3: Compensating charging voltage %.2fV -> estimated battery ~4.1V", voltage);
        return 4.1f;  // Conservative estimate during charging
    }
#else
    // T5 and other boards: ADC typically shows actual battery voltage even during charging
    // No compensation needed, but detect charging state for logging
    if (is_usb_charging()) {
        DLOG(TAG, "T5: Battery voltage during charging: %.2fV", voltage);
    }
#endif
    
    return voltage;  // Return actual reading
}

#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
/**
 * Diagnostic function to help identify ADC pin conflicts on LilyGO boards
 * Call this during initialization to check for potential issues
 */
void diagnose_adc_pin_conflicts(void) {
    ILOG(TAG, "=== ADC Pin Conflict Diagnostic (LilyGO Board) ===");
    ILOG(TAG, "ADC Channel: %d", CONFIG_ADC_CHANNEL);
    
    // Take multiple quick readings to check for stability
    float readings[5];
    bool stable = true;
    
    for (int i = 0; i < 5; i++) {
        readings[i] = volt_read();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Check for excessive variation (sign of interference)
    float min_v = readings[0], max_v = readings[0];
    for (int i = 1; i < 5; i++) {
        if (readings[i] < min_v) min_v = readings[i];
        if (readings[i] > max_v) max_v = readings[i];
    }
    
    float variation = max_v - min_v;
    if (variation > 0.5f) {  // > 500mV variation suggests interference
        stable = false;
        WLOG(TAG, "High voltage variation detected: %.3fV - possible pin conflict", variation);
    }
    
    ILOG(TAG, "Voltage readings: %.3f, %.3f, %.3f, %.3f, %.3f", 
         readings[0], readings[1], readings[2], readings[3], readings[4]);
    ILOG(TAG, "Variation: %.3fV, Stable: %s", variation, stable ? "YES" : "NO");
    
    if (!stable) {
        WLOG(TAG, "ADC pin may be shared with other peripherals:");
        WLOG(TAG, "- Check if SD card, sensors, or display power management use same pin");
        WLOG(TAG, "- Consider using ULP readings which may be more stable");
        WLOG(TAG, "- Verify pin configuration in board documentation");
    } else {
        ILOG(TAG, "ADC readings appear stable");
    }
    
    ILOG(TAG, "=== End ADC Diagnostic ===");
}
#endif

/**
 * Convert raw ADC reading to calibrated voltage (in volts)
 * This function can be used with any raw ADC value, including stored RTC values
 */
float adc_raw_to_voltage(uint32_t raw_adc_value) {
    // Use unified calibration logic
    uint32_t calibrated_voltage = calibrate_adc_raw(raw_adc_value);
    return (float)calibrated_voltage / 1000.0f; // Convert mV to V
}

/**
 * Get battery voltage optimized for display updates
 * Parameters: raw_adc_value (0 if not available), fallback_voltage, output pointer
 * Uses reference parameter to avoid return value overhead - more efficient for frequent calls
 */
void get_battery_voltage_for_display(uint32_t raw_adc_value, float fallback_voltage, float *voltage_out) {
#if defined(CONFIG_LOGGER_ADC_ENABLED)
    if (!voltage_out) return; // Safety check
    
    if (raw_adc_value > 0) {
        // Use provided raw value with efficient calibration
        *voltage_out = adc_raw_to_voltage(raw_adc_value);
    } else {
        // Fallback to provided voltage value
        *voltage_out = fallback_voltage;
    }
#else
    if (voltage_out) *voltage_out = 3.6f; // Default fallback when ADC disabled
#endif
}

uint8_t calc_bat_perc_v(float adc) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s] %.04f", __func__, adc);
#endif
    uint32_t kadc = adc * 10000, sv, step, v, v1;
    uint8_t i=0, ret = 0, perc=0;
    if(kadc<=v_graph_lipo[0]) {
        ret = 0;
    }
    else if(kadc<=v_graph_lipo[V_GRAPH_LIPO_LEN-1]){
        for(;i<V_GRAPH_LIPO_LEN;++i, perc+=5) { // 0-100%
            v=v_graph_lipo[i]; // 32700
            v1=v_graph_lipo[i+1]; // 36100
            if(kadc == v) { // 32700
                ret = perc;
                goto done;
            }
            else if(kadc == v1) { // 36100
                ret = perc+5;
                goto done;
            }
            else if(kadc<v1) { // between 32700 and 36100
                step = (v1 - v) / 5; // divide by 1% for steps
                ++perc;
                for(sv=v+step;sv<=v1;sv+=step,++perc) {
                    if(kadc<=sv) {
                        ret = perc;
                        goto done;
                    }
                }
            }
        }
    } else ret = 100;
    done:
#if (C_LOG_LEVEL < 1)
    DLOG(TAG,"[%s] voltage: %f converted: %lu mV perc: %hhu", __func__, adc, kadc, ret);
#endif
    return ret;
}

// uint32_t calc_bat_perc(float adc) {
//     ILOG(TAG, "[%s] %0.4f", __func__, adc);
//     uint32_t adck = adc * 1000;
//     uint32_t bat_perc = VOLTAGE_PERC(adck);
// #if (C_LOG_LEVEL < 1)
//     DLOG(TAG, "[%s] voltage: %f converted: %lu mV perc: %lu coef: %lu", __func__, adc, adck, bat_perc, VOLTAGE_PERC_COEF(adck));
// #endif
//     if (bat_perc < 0)
//         bat_perc = 0;
//     else if (bat_perc > 100)
//         bat_perc = 100;
//     return bat_perc;
// }
static const char * cali_mode = "";
static uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    esp_err_t ret = ESP_FAIL;
    uint8_t calibrated = false;
    adc_cali_handle_t handle = NULL;
    if (!calibrated) {
        
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
        cali_mode = "Curve Fitting";
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
        cali_mode = "Line Fitting";
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = _ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
#endif
#if (C_LOG_LEVEL < 2)
        DLOG(TAG,"[%s] calibration scheme version is %s", __func__, cali_mode);
#endif
        if (ret == ESP_OK) calibrated = true;
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
#if (C_LOG_LEVEL < 1)
        DLOG(TAG,"[%s] Calibration Success", __func__);
#endif
    } else 
    if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
#if (C_LOG_LEVEL < 3)
        WLOG(TAG, "[%s] eFuse not burnt, skip software calibration", __func__);
#endif
    } else {
        ELOG(TAG, "[%s] Invalid arg or no memory", __func__);
    }
    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if (C_LOG_LEVEL < 2)
    ILOG(TAG, "[%s]", __func__);
    DLOG(TAG, "[%s] deregister %s calibration scheme", __func__, cali_mode);
#endif
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_curve_fitting(handle)) {
        ELOG(TAG, "[%s] Failed to delete curve fitting scheme", __func__);
    }
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_line_fitting(handle)) {
        ELOG(TAG, "[%s] Failed to delete line fitting scheme", __func__);
    }
#endif
}

static uint32_t adc_read_raw() {
    esp_err_t err = 0;
    int v = 0;
#if defined(CONFIG_ULP_COPROC_ENABLED)
    // ULP mode: No direct ADC readings to avoid conflicts
    WLOG(TAG, "[%s] ULP mode active - direct ADC disabled to avoid unit conflicts", __func__);
    adc_ctx.adc_raw = 0;
    v = 0;
#elif defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    // Regular mode: Use direct ADC readings
    if(adc_oneshot_read(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &v)) {
        ELOG(TAG, "[%s] Failed to read ADC %d", __func__, _ADC_CHANNEL_0);
        return 0;
    }
    adc_ctx.adc_raw = v;
#endif
    if (adc_ctx.do_calibration) {
        if(adc_cali_raw_to_voltage(adc_ctx.adc1_cali_handle, adc_ctx.adc_raw, &v)) {
            ELOG(TAG, "[%s] Failed to convert", __func__);
            return 0;
        }
        adc_ctx.adc_voltage = v;
    }
    else adc_ctx.adc_voltage = adc_ctx.adc_raw;
    // TLOG(TAG, "[%s] ADC%d channel[%d]: raw: %lu, calibrated: %lu", __func__, _ADC_UNIT_0 + 1, _ADC_CHANNEL_0, adc_ctx.adc_raw, adc_ctx.adc_voltage);
    return adc_ctx.adc_voltage;
}

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)

static uint32_t adc_read_count(uint16_t count, uint16_t delay) {
    uint32_t readings[16]; // Buffer for up to 16 readings for median filtering
    uint32_t sum = 0;

    // Take multiple readings with settling time
    for (uint16_t i = 0; i < count; i++) {
        readings[i] = adc_read_raw();
        sum += readings[i];
        if (delay) vTaskDelay(pdMS_TO_TICKS(delay));
    }

    // Use median filtering to remove outliers (better than simple average for ADC noise)
    if (count >= 3) {
        // Sort readings for median calculation
        for (uint16_t i = 0; i < count - 1; i++) {
            for (uint16_t j = 0; j < count - i - 1; j++) {
                if (readings[j] > readings[j + 1]) {
                    uint32_t temp = readings[j];
                    readings[j] = readings[j + 1];
                    readings[j + 1] = temp;
                }
            }
        }
        // Return median value (middle element)
        uint32_t median = readings[count / 2];
        return median * 100;
    } else {
        // For small sample counts, use arithmetic mean
        return (sum / count) * 100;
    }
}
#if defined(AC_DETECTABLE ) && !(defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
static uint8_t result_avg_efficient() {
    uint8_t index = get_current_buffer_index();
    uint8_t prev_index = get_previous_buffer_index();
    if(adc_ctx.result_index >= RESULT_SIZE) adc_ctx.running_sum -= (adc_ctx.result[prev_index]);
    adc_ctx.running_sum += (adc_ctx.result[index]);
    if(adc_ctx.result_index >= RESULT_SIZE) {
        if(index == 0) {
            if(adc_ctx.m_avg[1]) adc_ctx.m_avg[2] = adc_ctx.m_avg[1]; // 2. RESULT_SIZE avg set
            if(adc_ctx.m_avg[0]) adc_ctx.m_avg[1] = adc_ctx.m_avg[0]; // 1. RESULT_SIZE avg set
            adc_ctx.m_avg[0] = adc_ctx.running_avg; // previous RESULT_SIZE avg set
            TLOG(TAG, "[%s] new set index 0, avg updated", __func__);
        }
        adc_ctx.running_avg = adc_ctx.running_sum / RESULT_SIZE;
        TLOG(TAG,"[%s] prev avg: {%lu, %lu, %lu}, avg: %lu, index: %hhu", __func__, adc_ctx.m_avg[2], adc_ctx.m_avg[1], adc_ctx.m_avg[0], adc_ctx.running_avg, index);
    }
    return (adc_ctx.m_avg[2] && adc_ctx.m_avg[0] > adc_ctx.m_avg[2]) ? 1 : 0;
}
#endif
static void adc_update(void*arg) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    // Take 11 readings with 5ms delay between each for better stability on LilyGO T5 charging circuits
    // Increased sample count and delay to handle rapid voltage fluctuations during charging
    uint32_t reading = VOLTAGE_CONV(adc_read_count(11, 5));
    if(xSemaphoreTake(adc_ctx.xMutex, pdMS_TO_TICKS(100))) {
        adc_ctx.result[++adc_ctx.result_index % RESULT_SIZE] = reading;
        xSemaphoreGive(adc_ctx.xMutex);
    } else {
        // Failed to take semaphore - log warning but continue
        WLOG(TAG, "[%s] Failed to take ADC mutex", __func__);
    }

    esp_event_post(ADC_EVENT, ADC_EVENT_UPDATE, &reading, sizeof(reading), pdMS_TO_TICKS(50));

    // Handle battery state machine for regular ADC mode (when ULP is disabled)
    // Convert from microvolts to millivolts using integer arithmetic to avoid float ops
    uint32_t voltage_mv_uint = reading / 1000;  // Integer division for performance
    float voltage_mv = (float)voltage_mv_uint;  // Only convert to float when needed
    handle_adc_battery_state(voltage_mv);

#if defined(AC_DETECTABLE)
    uint8_t on_ac = 0;
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    on_ac = gpio_get_level(GPIO_NUM_15);
#else
    on_ac = result_avg_efficient();
#endif
    if (on_ac != adc_ctx.on_ac) {
        adc_ctx.on_ac = on_ac;
        esp_event_post(ADC_EVENT, on_ac ? ADC_EVENT_CHARGE_STARTED : ADC_EVENT_CHARGE_STOPPED, &adc_ctx.on_ac, sizeof(adc_ctx.on_ac), portMAX_DELAY);
    }
#endif
}
#endif

uint8_t adc_on_ac() {
    return adc_ctx.on_ac;
}

#if defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_ctx.adc_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void adc_task(void * arg) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    esp_err_t ret;
    uint8_t count = 0;
    while (adc_ctx.task_is_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ret = adc_continuous_read(adc_ctx.adc1_handle, adc_ctx.result, READ_LEN, &adc_ctx.ret_num, 0);
        if (ret == ESP_OK) {
            for (int i = 0; i < adc_ctx.ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_ctx.result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);
                if (chan_num == SOC_ADC_CHANNEL_NUM(_ADC_CHANNEL_0)) adc_ctx.adc_raw = data;
            }
        }
        if (count > 10) count = 0;
        if (count == 0) {
            DLOG(TAG,"[%s] adc_raw: %lu", __func__, adc_ctx.adc_raw);
        }
        ++count;
        delay_ms(10);
    }
    vTaskDelete(NULL);
}

#endif

esp_err_t adc_init(void) {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    esp_err_t ret = 0;

    /* Initialize ULP coprocessor FIRST - ESP-IDF 5.x requires ULP ADC init before regular ADC */
#if defined(CONFIG_ULP_COPROC_ENABLED)
    esp_err_t ulp_err = init_ulp_program();
    if (ulp_err != ESP_OK) {
        WLOG(TAG, "ULP initialization failed: %s (continuing without ULP)", esp_err_to_name(ulp_err));
        /* Don't fail the entire init if ULP fails - graceful degradation */
    } else {
        ILOG(TAG, "ULP continuous battery monitoring enabled");
        
        // Wait longer for ULP to start running, then check multiple times
        for (int i = 0; i < 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait 500ms between checks
            uint32_t sample_count = ulp_sample_counter;
            uint32_t last_result = ulp_last_result;
            uint32_t rtc_mem_0 = RTC_SLOW_MEM[0];
            
            ILOG(TAG, "ULP check %d: sample_counter=%lu, last_result=%lu, RTC_SLOW_MEM[0]=%lu", 
                 i+1, sample_count, last_result, rtc_mem_0);
                 
            if (sample_count > 0 || rtc_mem_0 > 0) {
                ILOG(TAG, "ULP is executing successfully!");
                break;
            }
            
            if (i == 4) { // After 2.5 seconds, try resetting the ULP timer
                WLOG(TAG, "ULP not responding, attempting timer reset...");
                REG_SET_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                REG_SET_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 1);
                ILOG(TAG, "ULP timer reset complete");
            }
        }
    }
#endif

    // Setup calibration (works for both ULP and regular ADC)
    adc_ctx.do_calibration = adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN, &adc_ctx.adc1_cali_handle);

#if defined(CONFIG_ULP_COPROC_ENABLED)
    // When ULP is enabled, use ULP exclusively for battery monitoring
    ILOG(TAG, "ULP enabled - using ULP ADC for battery monitoring");
    // Regular ADC is not initialized when ULP is enabled due to channel conflicts
#else
    // When ULP is disabled, use regular ADC normally
    ILOG(TAG, "ULP disabled - using regular ADC");

    // Initialize regular ADC
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = _ADC_UNIT_0,
    };
    if(adc_oneshot_new_unit(&init_config1, &adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to create ADC unit", __func__);
        return ESP_FAIL;
    }
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = _ADC_BITWIDTH,
        .atten = _ADC_ATTEN,
    };
    if(adc_oneshot_config_channel(adc_ctx.adc1_handle, _ADC_CHANNEL_0, &adc_config)) {
        ELOG(TAG, "[%s] Failed to config ADC channel", __func__);
        return ESP_FAIL;
    }
#endif

    // Initialize periodic ADC tasks for regular readings
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    if(adc_ctx.xMutex == NULL) adc_ctx.xMutex = xSemaphoreCreateMutex();
    adc_update(0);
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &adc_update,
        .name = "periodic_adc",
        .arg = NULL
    };
    if(esp_timer_create(&periodic_timer_args, &adc_ctx.adc_periodic_timer)){
        ELOG(TAG, "[%s] Failed to create periodic timer", __func__);
        return ESP_FAIL;
    }
    if(esp_timer_start_periodic(adc_ctx.adc_periodic_timer, SEC_TO_US(1))) {
        ELOG(TAG, "[%s] Failed to start periodic timer", __func__);
        return ESP_FAIL;
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    memset(&adc_ctx.result[0], 0xcc, READ_LEN);
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 256,
        .conv_frame_size = READ_LEN,
    };
    if(adc_continuous_new_handle(&adc_config, &adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to create ADC unit", __func__);
        return ESP_FAIL;
    }
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
        .pattern_num = 1,
    };

    adc_digi_pattern_config_t adc_patterns[1] = {0};
    dig_cfg.pattern_num = 0;
    adc_patterns[0].atten = _ADC_ATTEN;
    adc_patterns[0].channel = _ADC_CHANNEL_0;
    adc_patterns[0].unit = _ADC_UNIT_0;
    adc_patterns[0].bit_width = _ADC_BITWIDTH;
    DLOG(TAG, "adc_patterns[0].atten is 0x%"PRIx8"", adc_patterns[0].atten);
    DLOG(TAG, "adc_patterns[0].channel is 0x%"PRIx8"", adc_patterns[0].channel);
    DLOG(TAG, "adc_patterns[0].unit is 0x%"PRIx8"", adc_patterns[0].unit);
    dig_cfg.adc_pattern = adc_patterns;
    if(adc_continuous_config(adc_ctx.adc1_handle, &dig_cfg)){
        ELOG(TAG, "[%s] Failed to config ADC continuous", __func__);
        return ESP_FAIL;
    }
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    xTaskCreatePinnedToCore(adc_task, "ADC Task", (8*256), NULL, 0, &adc_ctx.adc_task_handle, 0);
    if(adc_continuous_register_event_callbacks(adc_ctx.adc1_handle, &cbs, NULL)) {
        ELOG(TAG, "[%s] Failed to register event callbacks", __func__);
        return ESP_FAIL;
    }
    if(adc_continuous_start(adc_ctx.adc1_handle)) {
        ELOG(TAG, "[%s] Failed to start ADC continuous", __func__);
        return ESP_FAIL;
    }
    delay_ms(200);
#endif
#endif
    return ret;
}

esp_err_t adc_deinit() {
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "[%s]", __func__);
#endif
    esp_err_t err = 0;
    if (adc_ctx.do_calibration) {
        adc_calibration_deinit(adc_ctx.adc1_cali_handle);
    }
#if !defined(CONFIG_ULP_COPROC_ENABLED)
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    esp_timer_stop(adc_ctx.adc_periodic_timer);
    esp_timer_delete(adc_ctx.adc_periodic_timer);
    adc_oneshot_del_unit(adc_ctx.adc1_handle);
    if(adc_ctx.xMutex != NULL){
        vSemaphoreDelete(adc_ctx.xMutex);
        adc_ctx.xMutex = NULL;
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    adc_ctx.task_is_running = 0;
    xTaskNotifyGive(adc_ctx.adc_task_handle);
    adc_continuous_stop(adc_ctx.adc1_handle);
    adc_continuous_deinit(adc_ctx.adc1_handle);
#endif
#endif
    return err;
}

float volt_read(void) {
    ILOG(TAG, "[%s]", __func__);
    float voltage = 0;
    
#ifdef CONFIG_ULP_COPROC_ENABLED
    /* Use calibrated ULP measurement if available and valid */
    uint32_t ulp_calibrated_mv = get_ulp_adc_calibrated();
    if (ulp_calibrated_mv > 0) {
        /* ULP reading is already calibrated and validated, convert to volts */
        voltage = VOLTAGE_U32_TO_V((float)ulp_calibrated_mv);
        DLOG(TAG, "[%s] ULP calibrated: %lu mV, volt: %f", __func__, ulp_calibrated_mv, voltage);
        return voltage;
    }
    
    static uint32_t ulp_debug_counter = 0;
    ulp_debug_counter++;
    
    // Show detailed ULP status every 10 calls when readings aren't available
    if (ulp_debug_counter % 10 == 1) {
        debug_ulp_status();
    }
    
    // Run comprehensive ULP test every 50 calls (for deeper debugging)
    if (ulp_debug_counter % 50 == 25) {
        test_ulp_functionality();
    }
    
    DLOG(TAG, "[%s] No valid ULP reading available, using direct ADC", __func__);
#endif

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    // Use temporal smoothing: average last 3 readings from circular buffer for better stability
    if(xSemaphoreTake(adc_ctx.xMutex, pdMS_TO_TICKS(100))) {
        uint32_t sum = 0;
        uint8_t valid_readings = 0;
        int16_t idx = adc_ctx.result_index;

        // Average last 3 readings (or fewer if buffer not full)
        for (int8_t i = 0; i < 3 && i <= idx; i++) {
            int16_t buffer_idx = (idx - i) % RESULT_SIZE;
            if (buffer_idx < 0) buffer_idx += RESULT_SIZE; // Handle negative modulo
            uint32_t reading = adc_ctx.result[buffer_idx];
            if (reading > 0) { // Only include valid readings
                sum += reading;
                valid_readings++;
            }
        }

        xSemaphoreGive(adc_ctx.xMutex);

        if (valid_readings > 0) {
            voltage = VOLTAGE_U32_TO_V(((float)sum / valid_readings));
        } else {
            // Fallback to most recent reading if no valid readings found
            voltage = get_recent_voltage_reading();
        }
    } else {
        // Semaphore timeout, use most recent reading
        voltage = get_recent_voltage_reading();
    }
#elif defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)
    voltage = VOLTAGE_U32_TO_V((VOLTAGE_CONV((float)VOLTAGE_CONV_12(adc_ctx.adc_raw))));
#endif    
    
    // Additional validation for shared pin scenarios  
#if (defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3) || defined(CONFIG_HAS_BOARD_LILYGO_T_DISPLAY_S3_AMOLED))
    voltage = validate_and_clamp_voltage(voltage, true);
#else
    voltage = validate_and_clamp_voltage(voltage, false);
#endif

    DLOG(TAG, "[%s] adc_raw: %lu volt: %f", __func__, adc_ctx.adc_raw, voltage);
    return voltage;
}

#endif // CONFIG_LOGGER_ADC_ENABLED
