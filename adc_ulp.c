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

static const char *TAG = "adc_ulp";
RTC_DATA_ATTR bool ulp_initialized = false;

/* ULP binary references */
extern const uint8_t ulp_battery_bin_start[] asm("_binary_ulp_battery_bin_start");
extern const uint8_t ulp_battery_bin_end[]   asm("_binary_ulp_battery_bin_end");

/* ULP memory is 32-bit word addressed - all variables are uint32_t */
/* For small values, only lower bits are used */
extern uint32_t ulp_wake_data;
extern uint32_t ulp_cycle_count;
extern uint32_t ulp_low_thr;         /* Only lower 12 bits used */
extern uint32_t ulp_last_result;     /* Only lower 12 bits used */
extern uint32_t ulp_entry;

#ifdef CONFIG_ULP_BUTTON_ENABLED
extern uint32_t ulp_button_press_counter;
extern uint32_t ulp_button_last_result;
/* ULP st instruction always writes 32-bit, so .word packing doesn't work */
/* Must use .long and access as uint32_t */
#define ulp_button_press_counter_get() (ulp_button_press_counter & 0xFFFF)
#define ulp_button_press_counter_set(val) (ulp_button_press_counter = (val) & 0xFFFF)
#define ulp_button_last_result_get() (ulp_button_last_result & 0x1)
#define ulp_button_last_result_set(val) (ulp_button_last_result = (val) & 0x1)
#endif
#ifdef CONFIG_ULP_BATTERY_MONITORING_ENABLED
extern uint32_t ulp_rapid_change_thr;      /* Only lower 16 bits used */
extern uint32_t ulp_cum_change;            /* Only lower 16 bits used */
extern uint32_t ulp_prev_result[ULP_ADC_HISTORY_SIZE];  /* Each: only lower 12 bits used */
extern uint32_t ulp_prev_result_idx;       /* Only lower 16 bits used */
extern uint32_t ulp_sample_count;          /* Only lower 16 bits used */
extern uint32_t ulp_running_sum;           /* Full 32 bits used */
#endif
extern uint32_t ulp_debug_counter;  /* Only lower 16 bits used */

/* 
 * Safe ULP variable access macros
 * The ULP compiler generates uint32_t symbols and our assembly uses 32-bit words
 * These macros handle the type conversion safely
 */
#define ULP_GET_U32(var) (var & UINT16_MAX)
#define ULP_SET_U32(var, val) ((var) = (val))
#define ULP_GET_U16(var) (*(volatile uint16_t*)&(var) & UINT16_MAX)
#define ULP_SET_U16(var, val) (*(volatile uint16_t*)&(var) = (val))
#define ULP_GET_U8(var) (*(volatile uint8_t*)&(var) & UINT8_MAX)  
#define ULP_SET_U8(var, val) (*(volatile uint8_t*)&(var) = (val))
#define ULP_GET_ARR_U32(arr, i) (((volatile uint32_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U32(arr, i, val) (((volatile uint32_t*)&(arr))[i] = (val))
#define ULP_GET_ARR_U16(arr, i) (((volatile uint16_t*)&(arr))[i] & UINT16_MAX)
#define ULP_SET_ARR_U16(arr, i, val) (((volatile uint16_t*)&(arr))[i] = (val))

// Access prev_result array from ULP
#define ADC_THRESHOLD_TRIGGER 1

static void adc_ulp_init_rtc_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }
    // Configure button GPIO for ULP use
    rtc_gpio_init(rtc_gpio);
    rtc_gpio_set_direction(rtc_gpio, RTC_GPIO_MODE_INPUT_ONLY);
    switch(rtc_gpio) {
        case CONFIG_ULP_BUTTON_GPIO:
            rtc_gpio_pullup_en(rtc_gpio);
            rtc_gpio_pulldown_dis(rtc_gpio);
            break;
        default:
            rtc_gpio_pulldown_dis(rtc_gpio);
            rtc_gpio_pullup_dis(rtc_gpio);
            break;
    }
    //
    ILOG(TAG, "GPIO pin %d configured for RTC.", rtc_gpio);
}


static void adc_ulp_uninit_pin(int rtc_gpio)
{
    FUNC_ENTRY(TAG);
    if (rtc_gpio == -1) {
        return;
    }
    rtc_gpio_deinit(rtc_gpio);
    gpio_reset_pin(rtc_gpio);
    gpio_hold_dis(rtc_gpio);
    rtc_gpio_hold_dis(rtc_gpio);
    // rtc_gpio_force_hold_dis(rtc_gpio);
    ILOG(TAG, "GPIO pin %d cleared RTC.", rtc_gpio);
}

static void configure_adc_pad(void)
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
            ELOG(TAG, "Unsupported ADC channel %d", _ADC_CHANNEL_0);
            return;
    }
    
    // Configure ADC pad for ULP use
    adc_ulp_init_rtc_pin(adc_gpio);

    ILOG(TAG, "ADC GPIO%d (channel %d) configured for ULP", adc_gpio, _ADC_CHANNEL_0);
}

static void adc_ulp_init_pins(void)
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

esp_err_t init_ulp_program(void) {
    FUNC_ENTRY(TAG);
    esp_err_t err = ESP_OK;

    // Prevent re-initialization which would wipe ULP RAM and reset all state
    if (ulp_initialized) {
        WLOG(TAG, "ULP already initialized, skipping binary load to preserve state");
        return ESP_OK;
    }

    ILOG(TAG, "Initializing ULP program...");
    if(adc_lock(1000)) {
        adc_unlock();
    }
    // Reset and prepare ULP
    // ulp_timer_stop();
    // vTaskDelay(pdMS_TO_TICKS(50));

   // First, let's verify the ULP binary was loaded correctly

    const size_t ulp_prog_size_bytes = ulp_battery_bin_end - ulp_battery_bin_start;
    const size_t ulp_prog_size_words = ulp_prog_size_bytes / sizeof(uint32_t);

    err = ulp_load_binary(0, ulp_battery_bin_start, ulp_prog_size_words);

    if (err != ESP_OK) {
        ELOG(TAG, "Failed to load ULP program: %s", esp_err_to_name(err));
        goto error;
    }

    ulp_adc_cfg_t adc_cfg = {
        .adc_n = _ADC_UNIT_0,     // Use same unit as regular ADC
        .channel = _ADC_CHANNEL_0, // Use same channel as regular ADC  
        .atten = _ADC_ATTEN,      // Use same attenuation as regular ADC
        .width = _ADC_BITWIDTH,   // Use same bitwidth as regular ADC (only for ADC1)
        .ulp_mode = ADC_ULP_MODE_FSM, // Explicitly specify FSM mode for ESP32 (not RISC-V)
    };

    // Initialize ULP ADC
    err = ulp_adc_init(&adc_cfg);
    if (err != ESP_OK) {
        ELOG(TAG, "ULP ADC init failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_ulp_init_pins();

    // Initialize ULP variables - but DON'T reset sample_count, prev_result_idx, or cycle_count
    // These are managed by ULP assembly's first_run_init logic
    ULP_SET_U32(ulp_wake_data, 0);
    ULP_SET_U32(ulp_last_result, 0);
    ULP_SET_U32(ulp_low_thr, ADC_LOW_TRESHOLD);
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    ulp_button_press_counter = 0;  /* Clears both counter (lower 16) and last_result (upper 16) */
#endif
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    ULP_SET_U32(ulp_rapid_change_thr, ADC_RAPID_CHANGE_TRESHOLD);
    // DON'T reset sample_count - ULP assembly uses it to detect first run
    // ULP_SET_U32(ulp_sample_count, 0);
    ULP_SET_U32(ulp_cum_change, 0);
    // DON'T reset prev_result_idx - preserve history buffer state
    // ULP_SET_U32(ulp_prev_result_idx, 0);
    // DON'T reset running_sum - preserve running sum state
    // ULP_SET_U32(ulp_running_sum, 0);
    // DON'T clear prev_result array - preserve ADC history
    // for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
    //     ULP_SET_ARR_U32(ulp_prev_result, i, 0);
    // }
#endif
    ULP_SET_U32(ulp_debug_counter, 0);
    printf("Raw ULP variable check:\n");
    printf("  low_thr addr=%p, value=0x%08lX (%lu)\n", &ulp_low_thr, ULP_GET_U32(ulp_low_thr), ULP_GET_U32(ulp_low_thr));
    printf("  last_result addr=%p, value=0x%08lX (%lu)\n", &ulp_last_result, ULP_GET_U32(ulp_last_result), ULP_GET_U32(ulp_last_result));
    ulp_initialized = true;
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
    if(adc_lock(1000)) {
        adc_unlock();
    }
    
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Don't reset any ULP state variables here!
    // The ULP assembly manages its own state through first_run_init and running calculations
    // Resetting variables here breaks the running sum and causes state loss
    
    // REMOVED: cycle_count reset - needed for ADC timing continuity
    // REMOVED: cum_change reset - recalculated by ULP each cycle
    // REMOVED: button resets - ULP manages button state
    
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    // Only reset button counter if we're starting fresh, not on every wake
    // ULP_SET_U32(ulp_button_press_counter, 0);
    // ULP_SET_U32(ulp_button_last_result, 0);
#endif

    /* Start the program */
    esp_err_t err = ulp_run((uint32_t*)&ulp_entry - RTC_SLOW_MEM);
    if(err) {
        ELOG(TAG, "Failed to start ULP program: %s\n", esp_err_to_name(err));
        return;
    }
}

// Current state accessors
uint8_t adc_get_ulp_wake_source(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_CURRENT_SOURCE_MASK) >> ULP_WAKE_CURRENT_SOURCE_SHIFT;
}
// Last state accessors  
static inline uint8_t adc_get_ulp_last_wake_source(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_LAST_SOURCE_MASK) >> ULP_WAKE_LAST_SOURCE_SHIFT;
}

uint8_t adc_get_ulp_wake_reason(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_CURRENT_ADC_MASK) >> ULP_WAKE_CURRENT_ADC_SHIFT;
}

uint8_t adc_get_ulp_last_wake_reason(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_LAST_ADC_MASK) >> ULP_WAKE_LAST_ADC_SHIFT;
}

static inline uint8_t adc_get_ulp_button_wake_reason(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_CURRENT_BUTTON_MASK) >> ULP_WAKE_CURRENT_BUTTON_SHIFT;
}

static inline uint8_t adc_get_ulp_last_button_reason(void) {
    return (ULP_GET_U32(ulp_wake_data) & ULP_WAKE_LAST_BUTTON_MASK) >> ULP_WAKE_LAST_BUTTON_SHIFT;
}

// Your existing functions
bool adc_ulp_button_long_press_detected(void) {
    return (adc_get_ulp_wake_source() & ULP_WAKE_SOURCE_BUTTON) && 
           (adc_get_ulp_button_wake_reason() == ULP_BUTTON_WAKE_LONG_PRESS);
}

// Check if same as last ADC wake reason (for suppression)
bool adc_ulp_same_adc_wake_reason(void) {
    return (adc_get_ulp_wake_reason() != ULP_ADC_WAKE_NONE) &&
           (adc_get_ulp_wake_reason() == adc_get_ulp_last_wake_reason());
}

void adc_ulp_clear_wake_sources(void) {
    FUNC_ENTRY(TAG);
    ULP_SET_U32(ulp_wake_data, 0);
}

/**
 * Diagnostic function to debug ULP status
 */
void debug_ulp_status(void) {
#if (C_LOG_LEVEL < 3)
    if(!ulp_initialized) {
        return;
    }
    printf("=== ULP Diagnostic Status ===\n");
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    printf("ULP Button Press Counter addr=%p, value=%u (0x%04X)\n", 
           &ulp_button_press_counter, ulp_button_press_counter_get(), ulp_button_press_counter_get());
    printf("ULP Button Last Result addr=%p, value=%u (0x%04X)\n", 
           &ulp_button_last_result, ulp_button_last_result_get(), ulp_button_last_result_get());
    printf("ULP Button vars as uint32: counter=0x%08lX, last=0x%08lX\n",
           ulp_button_press_counter, ulp_button_last_result);
#endif
    printf("ULP Last Result: %lu\n", ULP_GET_U32(ulp_last_result));
    printf("ULP Low Threshold: %lu\n", ULP_GET_U32(ulp_low_thr));
    printf("ULP Cycle Count: %lu\n", ULP_GET_U32(ulp_cycle_count));
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    printf("ULP Rapid Change Threshold: %lu\n", ULP_GET_U32(ulp_rapid_change_thr));
    printf("ULP Sample Count: %lu\n", ULP_GET_U32(ulp_sample_count));
    printf("ULP Cumulative Change: %lu\n", ULP_GET_U32(ulp_cum_change));
    printf("ULP Prev Result Index: %lu\n", ULP_GET_U32(ulp_prev_result_idx));
    printf("ULP Running Sum: %lu\n", ULP_GET_U32(ulp_running_sum));
    printf("ULP Prev Result (raw): [");
    for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        printf("0x%04lX", ULP_GET_ARR_U32(ulp_prev_result, i));
        if (i < (ULP_ADC_HISTORY_SIZE - 1)) printf(", ");
    }
    printf("]\n");
    printf("ULP Prev Result: [");
    for (int i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        printf("%lu", ULP_GET_ARR_U32(ulp_prev_result, i));
        if (i < (ULP_ADC_HISTORY_SIZE - 1)) printf(", ");
    }
    printf("]\n");
#endif
    // Decode wake_data bits
    uint32_t wake_data = ULP_GET_U32(ulp_wake_data);
    uint8_t curr_source = (wake_data >> ULP_WAKE_CURRENT_SOURCE_SHIFT) & 0x3;
    uint8_t curr_adc = (wake_data >> ULP_WAKE_CURRENT_ADC_SHIFT) & 0x7;
    uint8_t curr_button = (wake_data >> ULP_WAKE_CURRENT_BUTTON_SHIFT) & 0x7;
    uint8_t last_source = (wake_data >> ULP_WAKE_LAST_SOURCE_SHIFT) & 0x3;
    uint8_t last_adc = (wake_data >> ULP_WAKE_LAST_ADC_SHIFT) & 0x7;
    uint8_t last_button = (wake_data >> ULP_WAKE_LAST_BUTTON_SHIFT) & 0x7;
    
    printf("ULP Wake Data: 0x%08lX\n", wake_data);
    printf("  Current: Source=%s, State: adc=%s, button=%hhu\n", adc_ulp_wake_sources_str[curr_source], adc_battery_states_str[curr_adc], curr_button);
    printf("  Last:    Source=%s, State: adc=%s, button=%hhu\n", adc_ulp_wake_sources_str[last_source], adc_battery_states_str[last_adc], last_button);
#if defined(CONFIG_ULP_BUTTON_ENABLED)
    printf("  Debug:   Button press in progress: counter=%lu (0=no press, >0=pressing, threshold=%d)\n", 
           ulp_button_press_counter_get(), ULP_LONG_PRESS_CYCLES);
#endif
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
    // Get ULP variables (raw ADC values, not voltage) - 32-bit word access with masking
    const uint32_t current_result = ULP_GET_U32(ulp_last_result) & 0xFFF;   // 12-bit ADC result (keep mask)
    const uint32_t low_thr = ADC_LOW_TRESHOLD;         // Low threshold from config
    // const uint32_t high_thr = ADC_HIGH_TRESHOLD;    // High threshold from config
    const uint32_t rapid_thr = ADC_RAPID_CHANGE_TRESHOLD;  // Rapid change threshold
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    uint32_t prev_result_avg = 0; // Previous reading average
    for (uint8_t i = 0; i < ULP_ADC_HISTORY_SIZE; i++) {
        prev_result_avg += ULP_GET_ARR_U32(ulp_prev_result, i) & 0xFFF;  // 12-bit ADC results (keep mask)
    }
    prev_result_avg /= ULP_ADC_HISTORY_SIZE; // Average previous readings
    const int32_t change = (int32_t)current_result - (int32_t)prev_result_avg;
#endif
    DLOG(TAG, "ULP state analysis: current=%lu, "
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
    "change=%ld, "
#endif
    "low_thr=%lu, rapid_thr=%lu",
         current_result,
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)
         change, 
#endif
         low_thr, rapid_thr);
    
    // Check for low battery condition (primary ULP function)
    if (current_result <= low_thr) {
        WLOG(TAG, "ULP: detected low battery.");
        return ADC_BATTERY_LOW;
    }
    
    // Check for high battery condition (battery full detection)
    // if (current_result >= high_thr) {
    //     ILOG(TAG, "ULP: detected high battery.");
    //     return ADC_BATTERY_HIGH;
    // }
#if defined(CONFIG_ULP_BATTERY_MONITORING_ENABLED)    
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
#endif
    // Default to normal state - ULP woke us but no specific condition detected
#if (C_LOG_LEVEL < 3)
    ILOG(TAG, "ULP battery state: normal.");
#endif
    return ADC_BATTERY_NORMAL;
}

#endif /* CONFIG_ULP_COPROC_ENABLED */