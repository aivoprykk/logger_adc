# logger_adc

This module provides ADC (Analog-to-Digital Converter) logging functionality for the ESP-IDF GPS Logger project.

## Features

- Reads analog values from specified ADC channels
- Configurable sampling rate and resolution
- Buffered logging for efficient data storage
- Integration with the main logger system

## Usage

1. **Include the module** in your component dependencies.
2. **Configure ADC parameters** in your project settings or code.
3. **Initialize the logger_adc** in your application:
    ```c
    #include "logger_adc.h"

    void app_main(void) {
         logger_adc_init();
         // Start logging
         logger_adc_start();
    }
    ```
4. **Retrieve or store logged data** as needed.

## Configuration

- ADC channel selection
- Sampling frequency
- Buffer size

Refer to the source code and comments for detailed configuration options.

## Dependencies

- ESP-IDF ADC driver
- Main logger component

## 📚 Developer Documentation

For detailed technical documentation, architecture guides, and implementation examples, see:

**[doc/development/adc/](../../doc/development/adc/)** - ADC Development Documentation
- **[ADC_CALIBRATION_GUIDE.md](../../doc/development/adc/ADC_CALIBRATION_GUIDE.md)** - ADC calibration and voltage conversion guide
  - Manual conversion formulas for 12-bit ADC
  - Why calibration fails in ULP/RTC mode
  - Voltage divider calculations and accuracy analysis
  
- **[ULP_ADC_REFACTOR_GUIDE.md](../../doc/development/adc/ULP_ADC_REFACTOR_GUIDE.md)** - ULP-as-primary ADC architecture guide
  - Dual-mode implementation patterns with preprocessor flags
  - Benefits analysis and migration checklist
  
- **[REFACTOR_EXAMPLE_volt_read.c](../../doc/development/adc/REFACTOR_EXAMPLE_volt_read.c)** - Working code example
  - volt_read() dual implementation (ULP primary vs regular ADC)
  - Helper functions and usage examples

## License

This module is licensed under the MIT License.
