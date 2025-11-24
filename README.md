# Logger ADC Component

A comprehensive ADC (Analog-to-Digital Converter) monitoring system for ESP32-based devices, designed for battery-powered applications requiring low-power operation and intelligent battery state management.

## Features

### Core Functionality
- **Dual ADC Operating Modes**:
  - One-shot mode using ESP-IDF's ADC driver
  - ULP coprocessor mode for ultra-low-power monitoring during deep sleep
- **Advanced Battery State Management** with four states: Normal, Critical Low, Charging Started, Charging Stopped
- **Intelligent Charging Detection** using adaptive delta + plateau algorithm
- **Battery Percentage Calculation** with configurable voltage curves
- **Safety Monitoring** with low battery callbacks and thresholds

### ULP Coprocessor Features
- **Low-Power Monitoring**: Continuous battery voltage monitoring during deep sleep
- **Wake-on-Event**: Automatic main CPU wake for critical events
- **Circular Buffer Averaging**: Rolling average to filter ADC noise
- **Adaptive Thresholds**: Dynamic voltage thresholds based on battery state
- **Cycle Counting**: ULP execution tracking for debugging

### Additional Features
- **Optional Button Monitoring**: ULP-based long-press detection
- **Event System Integration**: ESP Event framework support
- **Thread-Safe Operations**: Safe multi-task ADC data access
- **Comprehensive Configuration**: Extensive Kconfig options
- **Debugging Support**: ULP status inspection and performance metrics

## Installation

### ESP-IDF Integration
Add to your `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES logger_adc)
```

### PlatformIO
Add to your `platformio.ini`:
```ini
[env]
lib_deps =
    https://github.com/aivoprykk/esp-gps-logger.git#components/logger_adc
```

## Configuration

### Kconfig Options
Configure via `idf.py menuconfig` or PlatformIO:

- **LOGGER_ADC_ENABLED**: Enable/disable the ADC module
- **ADC_CYCLE_TIME_MS**: Timer cycle time (20-1000ms)
- **ULP_BATTERY_MONITORING_ENABLED**: Enable ULP battery monitoring
- **ULP_ADAPTIVE_DELTA_PLATEAU**: Use advanced charging detection algorithm
- **ULP_BUTTON_ENABLED**: Enable ULP button monitoring
- **ADC_UNIT/ADC_CHANNEL**: ADC hardware configuration
- **ADC_ATTEN/ADC_BITWIDTH**: ADC electrical parameters

### Hardware Setup
- Connect battery voltage through appropriate voltage divider
- Ensure ADC channel matches your hardware (GPIO36-39 for ESP32 ADC1)
- For ULP button: Use RTC-capable GPIO (see Kconfig for mappings)

## Usage

### Basic Initialization
```c
#include "adc.h"

// Initialize ADC module
if (adc_init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC");
    return;
}

// Get battery percentage
uint8_t percentage = adc_calc_bat_perc(adc_get_cached_batt_volt());
ESP_LOGI(TAG, "Battery: %d%%", percentage);
```

### ULP Mode Setup
```c
// Initialize ULP program (call once at startup)
if (init_ulp_program() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load ULP program");
    return;
}

// Start ULP ADC monitoring
if (init_ulp_adc() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ULP ADC");
    return;
}
```

### Event Handling
```c
// Register for battery events
esp_event_handler_register(ADC_EVENT, ESP_EVENT_ANY_ID, adc_event_handler, NULL);

static void adc_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    switch (event_id) {
        case ADC_EVENT_NORMAL:
            ESP_LOGI(TAG, "Battery normal");
            break;
        case ADC_EVENT_CRITICAL_LOW:
            ESP_LOGI(TAG, "Battery critical!");
            break;
        case ADC_EVENT_CHARGING:
            ESP_LOGI(TAG, "Charging started");
            break;
    }
}
```

### Battery State Queries
```c
// Get current battery state
adc_battery_state_t state = battery_get_current_battery_state();

// Check if charging
bool charging = adc_is_charging();

// Get voltage in millivolts
uint32_t voltage_mv = adc_get_cached_batt_mv();
```

## API Reference

### Core Functions
- `adc_init()` / `adc_deinit()`: Initialize/deinitialize ADC module
- `adc_calc_bat_perc(float voltage)`: Convert voltage to battery percentage
- `adc_get_cached_batt_volt()`: Get cached battery voltage
- `adc_is_charging()`: Check charging state

### ULP Functions
- `init_ulp_program()`: Load ULP binary
- `init_ulp_adc()` / `deinit_ulp_adc()`: ULP ADC control
- `adc_ulp_after_wake()`: Process ULP wake events
- `resume_ulp_program()`: Resume ULP after wake

### Battery State Functions
- `battery_get_current_battery_state()`: Get current state
- `battery_get_last_battery_state()`: Get previous state
- `battery_get_pending_battery_event()`: Check for pending events

### Event Suppression
- `adc_suppress_events()` / `adc_resume_events()`: Control event generation
- `adc_should_suppress_event()`: Check suppression status

## Battery States

| State | Value | Description |
|-------|-------|-------------|
| NORMAL | 0 | Battery within normal range |
| CRITICAL_LOW | 1 | Battery critically low - immediate action needed |
| CHARGING | 2 | Charging detected (rapid voltage increase) |
| CHARGING_STOPPED | 3 | Charging stopped (plateau/drop detected) |

## Power Consumption

### ULP Mode Benefits
- **Deep Sleep Monitoring**: Battery monitoring continues during sleep
- **Event-Driven Wake**: CPU only wakes for important events
- **Adaptive Sleep**: Longer sleep cycles when battery critical
- **Typical Consumption**: <50μA during monitoring (vs mA for active CPU)

### Configuration Impact
- **ADC_CYCLE_TIME_MS**: Lower values = more responsive but higher power
- **ULP_ADC_HISTORY_SIZE**: Larger buffers = better averaging but more memory
- **ULP_ADAPTIVE_DELTA_PLATEAU**: Advanced algorithm = more accurate but more processing

## Troubleshooting

### Common Issues
1. **ULP Not Waking**: Check RTC GPIO configuration and voltage divider
2. **False Charging Events**: Adjust adaptive thresholds or disable ADP algorithm
3. **High Power Consumption**: Verify deep sleep entry and ULP program loading

### Debug Commands
```c
// Show ULP status
debug_ulp_status();

// Get cycle count
uint32_t cycles = adc_ulp_get_cycle_count();
```

### Log Levels
Configure via Kconfig:
- TRACE: Detailed ULP execution info
- DEBUG: General debug information
- INFO: Important events only
- ERROR: Critical issues only

## Development

### Building ULP Assembly
ULP code is automatically compiled via `ulp_build.py` during build process. Manual compilation:
```bash
python ulp_build.py
```

### Testing
Run included examples:
```bash
cd examples/adc_demo
idf.py build flash monitor
```

### Contributing
- Follow ESP-IDF coding standards
- Add unit tests for new features
- Update documentation for API changes
- Test on multiple ESP32 variants

## Dependencies

- ESP-IDF v4.4+
- logger_common component
- esp_adc drivers
- FreeRTOS (for one-shot mode)

## License

See LICENSE file in component directory.

