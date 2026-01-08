#include "adc_private.h"

#if defined(CONFIG_LOGGER_ADC_ENABLED)
#include "esp_timer.h"
#include "esp_adc/adc_cali_scheme.h"

#include "adc_snapshot.h"
#include "ulp_config.h"
#include "main.h"  // For should_filter_charge_events

static const char *TAG = "adc_base";

static bool adc_initialized = false;

adc_context_t adc_ctx = ADC_CONTEXT_DEFAULT;
RTC_DATA_ATTR static uint32_t s_cached_batt_mv = 0;
RTC_DATA_ATTR static uint32_t rtc_stored_low_raw = 0;     // persists across deep-sleep (but NOT power-off)
RTC_DATA_ATTR static uint32_t rtc_stored_high_raw = 0;    // for hysteresis (clear threshold)

#define ADC_SUPPRESSION_TIMEOUT_MS SEC_TO_MS(5)  // 5 seconds max suppression
#define LOW_BAT_SEQUENCE_TIME_MS SEC_TO_MS(20)  // 20 seconds in milliseconds

static const uint16_t lipo_perc_table[] = {
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
#define arr_size(x) (sizeof(x)/sizeof((x)[0]))
ESP_EVENT_DEFINE_BASE(ADC_EVENT);
const char * const _nums[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };
inline const char * nums(int i) { return i < arr_size(_nums) ? _nums[i] : "?"; };
#if (C_LOG_LEVEL <= LOG_INFO_NUM)
static const char * const _adc_battery_states_str[] = { ADC_BAT_STATES(STRINGIFY) };
const char * adc_battery_states_str(int i) { return i >= arr_size(_adc_battery_states_str) ? "UNKNOWN_STATE" : _adc_battery_states_str[i]; };
const char * adc_event_strings(int id) { return id >= arr_size(_adc_battery_states_str) ? "UNKNOWN_EVENT" : _adc_battery_states_str[id]; }
#if defined(CONFIG_ULP_BUTTON_ENABLED)
static const char * const _adc_wake_sources_str[] = { ADC_WAKE_SOURCES(STRINGIFY) };
const char * adc_wake_sources_str(int i) { return i >= arr_size(_adc_wake_sources_str) ? nums(i) : _adc_wake_sources_str[i]; };
static const char * const _adc_button_wake_reasons_str[] = { ADC_BUTTON_STATES(STRINGIFY) };
const char * adc_button_wake_reasons_str(int i) { return i >= arr_size(_adc_button_wake_reasons_str) ? nums(i) : _adc_button_wake_reasons_str[i]; };
#endif
#else
const char * adc_battery_states_str(int i) {
#if (C_LOG_LEVEL <= LOG_ERR_NUM)
    if(i==ADC_BATTERY_CRITICAL_LOW)  return "CRITICAL_LOW";
#endif
    else return nums(i);
}
const char * adc_event_strings(int id) { return "ADC_EVENT"; }
#if defined(CONFIG_ULP_BUTTON_ENABLED)
const char * adc_wake_sources_str(int i) { return nums(i); }
const char * adc_button_wake_reasons_str(int i) { return nums(i); }
#endif
#endif

#define TIMEOUT_MAX portMAX_DELAY
static const TickType_t timeout_immediate = 0;
#define RESULT_MASK (RESULT_SIZE - 1)

bool adc_lock(int timeout) {
    if (!adc_ctx.xMutex) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? TIMEOUT_MAX : 
                                     (timeout == 0) ? timeout_immediate : pdMS_TO_TICKS(timeout);
    return xSemaphoreTake(adc_ctx.xMutex, timeout_ticks) == pdTRUE;
}

void adc_unlock() {
    if (adc_ctx.xMutex) xSemaphoreGive(adc_ctx.xMutex);
}

bool bat_safe_lock(int timeout) {
    if (!adc_ctx.batMutex) return false;
    const TickType_t timeout_ticks = (timeout == -1) ? TIMEOUT_MAX :
                                     (timeout == 0) ? timeout_immediate : pdMS_TO_TICKS(timeout);
    return  xSemaphoreTake(adc_ctx.batMutex, timeout_ticks) == pdTRUE;
}

void bat_safe_unlock() {
    if (adc_ctx.batMutex) xSemaphoreGive(adc_ctx.batMutex);
}

static const char * cali_mode = "";

/* Conversion wrapper: converts raw->mV using adc_cali */
esp_err_t raw_to_mv_wrapper(int raw, uint32_t *voltage_mv)
{
    if (adc_ctx.do_calibration) {
        int tmp = 0;
        esp_err_t ret = adc_cali_raw_to_voltage(adc_ctx.cali_handle, raw, &tmp);
        if (ret == ESP_OK && voltage_mv) {
            *voltage_mv = (uint32_t)tmp;
        }
        return ret;
    }
    return ESP_ERR_INVALID_STATE;
}

uint32_t find_raw_for_pin_mv(uint32_t pin_mv)
{
    FUNC_ENTRYD(TAG);
    uint32_t lo = 0;
    uint32_t hi = ADC_MAX_RAW;
    while (lo < hi) {
        uint32_t mid = (lo + hi) >> 1;
        uint32_t mv = 0;
        if (raw_to_mv_wrapper((int)mid, &mv) != ESP_OK) {
            // fallback: treat unknown as using linear scaling with DEFAULT_VREF to avoid infinite loop
            mv = (uint32_t)((uint64_t)mid * DEFAULT_VREF / ADC_MAX_RAW);
        }
        if (mv < pin_mv) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

uint32_t calibrate_adc_raw(uint32_t raw_reading) {
    if (raw_reading == 0) {
        return 0; // Invalid reading
    }
    
    uint32_t cal_reading = 0;

    /* Regular ADC Mode: Use hardware calibration if available */
    uint32_t cal = 0;
    if (raw_to_mv_wrapper((int)raw_reading, &cal) == ESP_OK) {
        cal_reading = cal;
    } else {
        ELOG(TAG, "ADC calibration not available, using manual conversion");
        cal_reading = raw_reading;
    }

    FUNC_ENTRY_ARGSD(TAG, "raw_reading=%lu, battery_mv=%lu", raw_reading, cal_reading);
    return cal_reading;
}

esp_err_t compute_and_store_thresholds(void) {
    FUNC_ENTRY(TAG);
    adc_calibration_init(_ADC_UNIT_0, _ADC_CHANNEL_0, _ADC_ATTEN);
    if (is_calibration_applied() && current_calibration.voltage_3V2) return ESP_OK;
    uint32_t raw_thresh = rtc_stored_low_raw;
    if (raw_thresh) goto calc;

    // 2) Compute adc pin voltage (after divider)
    // In your repo: HIGH_RESISTOR, LOW_RESISTOR (both in ohms)
    // 3) find the raw ADC value that maps to vpin_mv
    raw_thresh = find_raw_for_pin_mv(VOLTAGE_CONV_MV_TO_ADC_ULL(BATTERY_CRITICAL_LOW_MV));

    // 4) compute hysteresis/clear threshold (example: 5% above)
    // uint32_t raw_clear = raw_thresh + (raw_thresh * HYSTERESIS_PERCENT) / 100;
    // if (raw_clear > ADC_MAX_RAW) raw_clear = ADC_MAX_RAW;

    // 5) Calculate battery thresholds based on the critical low threshold
    // BATTERY_LOW_MV = 3400mV, BATTERY_HIGH_MV = 4150mV, desired_batt_mv = 3220mV
    // uint32_t battery_low_raw = (raw_thresh * 3400UL) / desired_batt_mv;
    // uint32_t battery_high_raw = (raw_thresh * 4150UL) / desired_batt_mv;

    // 6) persist in RTC slow memory so it survives deep-sleep (use NVS if you need across power cycles)
    rtc_stored_low_raw = raw_thresh;
    rtc_stored_high_raw = find_raw_for_pin_mv(VOLTAGE_CONV_MV_TO_ADC_ULL(4200UL));

    calc:
    apply_battery_calibration(rtc_stored_low_raw, rtc_stored_high_raw);

    // 7) ALSO write into ULP RAM symbol before starting ULP (see next snippet)
    ILOG(TAG, "Computed ULP thresholds: critical_low=%lu (%lumV), critical_high=%lu (%lumV)",
            raw_thresh, BATTERY_CRITICAL_LOW_MV, rtc_stored_high_raw, 4200UL);
    return ESP_OK;
}

static inline float adc_mv_to_voltage(uint32_t raw_adc_value) {
    return VOLTAGE_CONV_MV_TO_V(raw_adc_value); // Convert mV to V
}

static inline uint32_t adc_raw_to_mv(uint32_t raw_adc_value) {
    return VOLTAGE_CONV_ADC_TO_MV_UL(raw_adc_value); // Convert ADC reading to mV
}

static void handle_battery_state(adc_battery_state_t state) {
    FUNC_ENTRY_ARGS(TAG, "state: %d", state);
    // Check if we should filter events based on app mode
    if (should_filter_charge_events()) {
        // During boot/shutdown - only allow critical events to pass through
        if (state != ADC_BATTERY_CRITICAL_LOW) {
            WLOG(TAG, "charge event filtered during app mode transition: state=%d", state);
            return;
        }
    }

    // Check if this specific event should be suppressed
    if (adc_should_suppress_event(state)) {
        ILOG(TAG, "%s event suppressed during system transition: %s", __func__, adc_event_strings(state));
        return;
    }
    
    // Post the event
    float current_voltage = adc_mv_to_voltage(s_cached_batt_mv); 
    switch (state) {
        case ADC_BATTERY_CHARGING:
            ILOG(TAG, "%s detected %s: %.02f", __func__, adc_battery_states_str(ADC_BATTERY_CHARGING), current_voltage);
            adc_ctx.lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGING, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CHARGING_STOPPED:
            ILOG(TAG, "%s detected %s: %.02f", __func__, adc_battery_states_str(ADC_BATTERY_CHARGING_STOPPED), current_voltage);
            adc_ctx.lcd_charge_notification = true;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CHARGING_STOPPED, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(100));
            break;
        case ADC_BATTERY_CRITICAL_LOW:
            ELOG(TAG, "%s detected %s: %.02f", __func__, adc_battery_states_str(ADC_BATTERY_CRITICAL_LOW), current_voltage);
            adc_ctx.lcd_charge_notification = false;  // Notify LCD of charge event
            esp_event_post(ADC_EVENT, ADC_EVENT_CRITICAL_LOW, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(100));
            break;
        default:
            // Normal state - no event needed
            WLOG(TAG, "%s detected unknown battery state: %d", __func__, state);
            break;
    }
}

static adc_battery_state_t last_reported_state = ADC_BATTERY_NORMAL;  // Track last state we reported
// ADC update function called periodically from timer to read voltage and handle state
static void adc_update(void*arg) {
    FUNC_ENTRYD(TAG);
    uint32_t cal_reading = 0, raw_reading = 0;

#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    if(adc_lock(100)) {
        raw_reading = read_battery_adc();
        adc_unlock();
    }
#endif
    battery_snapshot_t* snap = battery_update_snapshot(raw_reading);
    cal_reading = calibrate_adc_raw(snap->battery_monitor.voltage_raw);
    s_cached_batt_mv = adc_raw_to_mv(cal_reading);
    FUNC_ENTRY_ARGSD(TAG, "got reading:%lu, converted_to_mv:%lu", cal_reading, s_cached_batt_mv);
    // Integrated low battery monitoring and RTC voltage update - runs with every ADC update
    if (bat_safe_lock(10)) {
        float current_voltage = adc_mv_to_voltage(s_cached_batt_mv);  // Convert millivolts to volts
        esp_event_post(ADC_EVENT, ADC_EVENT_UPDATE, &current_voltage, sizeof(current_voltage), pdMS_TO_TICKS(50));
        
        // Only handle battery state changes, not every cycle
        adc_battery_state_t current_state = battery_get_current_battery_state();
        if (current_state != last_reported_state) {
            adc_battery_state_t event_to_fire = current_state;
            
            // Special case: transition from CHARGING_STARTED to NORMAL means CHARGING_STOPPED
            if ((current_state == ADC_BATTERY_NORMAL && last_reported_state == ADC_BATTERY_CHARGING)) {
                event_to_fire = ADC_BATTERY_CHARGING_STOPPED;
            }
            
            if (event_to_fire != ADC_BATTERY_NORMAL) {
                handle_battery_state(event_to_fire);
            }
            last_reported_state = current_state;
        }
        
        // Low battery monitoring - trigger shutdown callback when battery is critically low
        if (snap->battery_monitor.voltage_raw < current_calibration.voltage_3V2) {
            uint32_t ms = get_millis();
            if (adc_ctx.low_bat_start_time_ms == 0) {
                adc_ctx.low_bat_start_time_ms = ms;
                ELOG(TAG, "Low battery detected: %lu mV < %lu mV - starting countdown", 
                     s_cached_batt_mv, BATTERY_CRITICAL_LOW_MV);
            } else if ((ms - adc_ctx.low_bat_start_time_ms) > LOW_BAT_SEQUENCE_TIME_MS) {
                ELOG(TAG, "Battery critically low for %d seconds - triggering shutdown", 
                     (int)FROM_K_UL(LOW_BAT_SEQUENCE_TIME_MS));
                if (adc_ctx.low_battery_callback) {
                    adc_ctx.low_battery_callback();
                }
                adc_ctx.low_bat_start_time_ms = 0; // Reset to avoid repeated calls
            }
        } else {
            adc_ctx.low_bat_start_time_ms = 0;
        }

        bat_safe_unlock();
    }
}

#ifdef USE_CLAMPED_VOLTAGE
float validate_and_clamp_voltage_mv(uint32_t voltage_mv) {
    uint32_t result;
    if (voltage_mv > USB_VOLTAGE_MAX_MV) goto fallback;
    if (voltage_mv < BATTERY_VOLTAGE_MIN_MV) goto fallback;
    if (voltage_mv > CHARGING_VOLTAGE_THRESHOLD_MV) {}
    goto end;
fallback:
    return VOLTAGE_CONV_MV_TO_V(FALLBACK_VOLTAGE_MV);
end:
    return VOLTAGE_CONV_MV_TO_V(voltage_mv);
}
#endif

float adc_get_cached_batt_volt(void) {
    FUNC_ENTRY(TAG);
    float voltage;
#if !defined(CONFIG_LOGGER_ADC_MODE_CONTINUOUS)    
    // Validate reading against board-specific thresholds
#ifdef USE_CLAMPED_VOLTAGE
    voltage = validate_and_clamp_voltage_mv(s_cached_batt_mv);
#else
    voltage = VOLTAGE_CONV_MV_TO_V(s_cached_batt_mv);
#endif
#else
    voltage = VOLTAGE_CONV_V(adc_ctx.adc_raw);
#endif
    battery_snapshot_t* snap = battery_get_snapshot();
    FUNC_ENTRY_ARGSD(TAG, "adc_raw: %hu, cali_v: %lu, volt: %f", snap->battery_monitor.voltage_raw, s_cached_batt_mv, voltage);
    return voltage;
}

/* Battery monitoring API - thread-safe access to battery data */
bool adc_check_battery_level(void) {
    if (!adc_initialized) return true; // Default to safe if not initialized
    FUNC_ENTRY_ARGS(TAG, " voltage_raw: %hu, threshold_3V2: %hu",
                    battery_get_snapshot()->battery_monitor.voltage_raw,
                    current_calibration.voltage_3V2);
    return (battery_get_snapshot()->battery_monitor.voltage_raw >= current_calibration.voltage_3V2);
}

void adc_set_low_battery_callback(void (*callback)(void)) {
    if (bat_safe_lock(50)) {
        adc_ctx.low_battery_callback = callback;
        bat_safe_unlock();
    } else {
        // Fallback assignment without mutex
        adc_ctx.low_battery_callback = callback;
    }
}

/**
 * Get battery voltage optimized for display updates
 * Parameters: raw_adc_value (0 if not available), fallback_voltage, output pointer
 * Uses reference parameter to avoid return value overhead - more efficient for frequent calls
 */
void get_battery_voltage_for_display(float *voltage_out) {
#if defined(CONFIG_LOGGER_ADC_ENABLED)
    if (!voltage_out) return; // Safety check
    *voltage_out = adc_mv_to_voltage(s_cached_batt_mv > 0 ? s_cached_batt_mv : FALLBACK_VOLTAGE_MV);
#else
    if (voltage_out) *voltage_out = 3.6f; // Default fallback when ADC disabled
#endif
}

/* ADC event suppression functions - prevent false events during system transitions */
void adc_suppress_events(const char* reason) {
    adc_ctx.events_suppressed = true;
    adc_ctx.suppression_start_time_ms = FROM_K_UL(esp_timer_get_time());  // Convert to ms
    ILOG(TAG, "[%s] ADC events suppressed: %s", __func__, reason);
}

void adc_resume_events(const char* reason) {
    adc_ctx.events_suppressed = false;
    adc_ctx.suppression_start_time_ms = 0;
    ILOG(TAG, "[%s] ADC events resumed: %s", __func__, reason);
}

bool adc_should_suppress_event(int event_id) {
    if (!adc_ctx.events_suppressed) {
        return false;
    }
    
    // Auto-resume after timeout to prevent permanent suppression
    int64_t current_time = FROM_K_UL(esp_timer_get_time());
    if (current_time - adc_ctx.suppression_start_time_ms > ADC_SUPPRESSION_TIMEOUT_MS) {
        WLOG(TAG, "[%s] ADC suppression timeout, auto-resuming", __func__);
        adc_resume_events("timeout");
        return false;
    }
    
    // Suppress charge-related events during transitions (state changes are prevented at source)
    if (event_id == ADC_EVENT_CHARGING || event_id == ADC_EVENT_CHARGING_STOPPED) {
        DLOG(TAG, "[%s] Suppressing ADC charge event during transition: %s", __func__, adc_event_strings(event_id));
        return true;
    }
    
    // Allow critical battery events to pass through
    if (event_id == ADC_EVENT_CRITICAL_LOW) {
        WLOG(TAG, "[%s] Allowing critical battery event despite suppression", __func__);
        return false;
    }
    
    // Suppress other battery state changes during transitions
    return true;
}

/* LCD charge notification flag - check and clear atomically */
bool adc_check_and_clear_lcd_charge_flag(void) {
    bool was_set = adc_ctx.lcd_charge_notification;
    if (was_set) {
        adc_ctx.lcd_charge_notification = false;
        DLOG(TAG, "[%s] LCD charge notification flag cleared", __func__);
    }
    return was_set;
}

uint8_t adc_calc_bat_perc(float adc) {
    FUNC_ENTRY_ARGS(TAG, "%.04f", adc);
    uint32_t kadc = adc * 10000, sv, step, v, v1, len = sizeof(lipo_perc_table)/sizeof(lipo_perc_table[0]);
    uint8_t i=0, ret = 0, perc=0;
    if(kadc<=lipo_perc_table[0]) {
        ret = 0;
    }
    else if(kadc<=lipo_perc_table[len-1]){
        for(;i<len;++i, perc+=5) { // 0-100%
            v=lipo_perc_table[i]; // 32700
            v1=lipo_perc_table[i+1]; // 36100
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
    DLOG(TAG,"[%s] voltage: %f converted: %lu mV perc: %hhu", __func__, adc, kadc, ret);
    return ret;
}

/* Initialize ADC calibration */
uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten) 
{
    FUNC_ENTRY(TAG);
    esp_err_t ret = ESP_FAIL;
    if (adc_ctx.cali_handle) return true;

#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    cali_mode = "Curve Fitting";
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = _ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_ctx.cali_handle);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    cali_mode = "Line Fitting";
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = _ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_ctx.cali_handle);
#endif
    ILOG(TAG,"[%s] calibration scheme version is %s", __func__, cali_mode);
    if (ret == ESP_OK) {
        ILOG(TAG,"[%s] Calibration Success", __func__);
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        WLOG(TAG, "[%s] eFuse not burnt, skip software calibration", __func__);
    } else {
        ELOG(TAG, "[%s] Invalid arg or no memory", __func__);
    }
    return !ret && adc_ctx.cali_handle && (adc_ctx.do_calibration = true);
}

/* Deinitialize ADC calibration */
void adc_calibration_deinit(void) {
#if (C_LOG_LEVEL > LOG_DEBUG_NUM)
    FUNC_ENTRY(TAG);
#else
    FUNC_ENTRY_ARGSD(TAG, " deregister %s calibration scheme", cali_mode);
#endif
    if(!adc_ctx.do_calibration) return;
    adc_ctx.do_calibration = false;
    if(!adc_ctx.cali_handle) return;
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_curve_fitting(adc_ctx.cali_handle)) {
        ELOG(TAG, "[%s] Failed to delete curve fitting scheme", __func__);
    }
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    if(adc_cali_delete_scheme_line_fitting(adc_ctx.cali_handle)) {
        ELOG(TAG, "[%s] Failed to delete line fitting scheme", __func__);
    }
#endif
    adc_ctx.cali_handle = NULL;
}


esp_err_t adc_init(void) {
    FUNC_ENTRY(TAG);
    if(adc_initialized) return ESP_OK; // Already initialized
    esp_err_t ret = 0;

    // Setup mutex for thread-safe ADC access
    if(adc_ctx.xMutex == NULL) adc_ctx.xMutex = xSemaphoreCreateMutex();
    if(adc_ctx.xMutex == NULL) {
        ELOG(TAG, "[%s] Failed to create mutex", __func__);
        return ESP_FAIL;
    }

    battery_snapshot_init();
    compute_and_store_thresholds();

#ifdef ULP_MODE
    ILOG(TAG, "ULP as primary ADC source - using manual voltage conversion");
    resume_ulp_program();
#elif defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_init();
#endif
    // Initialize periodic ADC tasks for regular readings
    // adc_update(0);
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &adc_update,
        .name = "periodic_adc",
        .arg = NULL
    };
    if(esp_timer_create(&periodic_timer_args, &adc_ctx.adc_timer)){
        ELOG(TAG, "[%s] Failed to create periodic timer", __func__);
        return ESP_FAIL;
    }
    if(esp_timer_start_periodic(adc_ctx.adc_timer, MS_TO_US(ADC_UPDATE_INTERVAL_MS))) {
        ELOG(TAG, "[%s] Failed to start periodic timer", __func__);
        return ESP_FAIL;
    }

    // Initialize battery safety mutex
    if (!adc_ctx.batMutex) {
        adc_ctx.batMutex = xSemaphoreCreateMutex();
        if (!adc_ctx.batMutex) {
            ELOG(TAG, "Failed to create battery safety mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    adc_initialized = true;
    return ret;
}

esp_err_t adc_deinit() {
    FUNC_ENTRY(TAG);
    if(!adc_initialized) return ESP_OK; // Not initialized
    adc_initialized = false;
    esp_err_t err = 0;
    if(adc_lock(-1)) {
        adc_unlock();
    }
    if (adc_ctx.adc_timer) {
        esp_timer_stop(adc_ctx.adc_timer);
        esp_timer_delete(adc_ctx.adc_timer);
        adc_ctx.adc_timer = NULL;
    }
#if defined(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    adc_oneshot_deinit();
#endif
    if(adc_ctx.xMutex != NULL){
        vSemaphoreDelete(adc_ctx.xMutex);
        adc_ctx.xMutex = NULL;
    }
    
    // Cleanup low battery timer
    if (adc_ctx.low_bat_timer) {
        esp_timer_stop(adc_ctx.low_bat_timer);
        esp_timer_delete(adc_ctx.low_bat_timer);
        adc_ctx.low_bat_timer = NULL;
    }

    // Cleanup battery safety mutex
    if(adc_ctx.batMutex != NULL){
        vSemaphoreDelete(adc_ctx.batMutex);
        adc_ctx.batMutex = NULL;
    }
    adc_ctx.low_battery_callback = NULL;

    adc_calibration_deinit();
    return err;
}

#endif // CONFIG_LOGGER_ADC_ENABLED
