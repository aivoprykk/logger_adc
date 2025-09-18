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

## License

This module is licensed under the MIT License.
