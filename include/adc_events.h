#ifndef E49D2157_B2DD_488F_BD1E_79BD2A2D925F
#define E49D2157_B2DD_488F_BD1E_79BD2A2D925F

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "logger_common.h"
#include "adc_defs.h"

#define ADC_EVENT_BASE 0x10  // Component ID 1

// Declare an event base
ESP_EVENT_DECLARE_BASE(ADC_EVENT);        // declaration of the ADC_EVENT family
#define ADC_EVENT_ENUM(l) ADC_EVENT_##l,
#define ADC_EVENT_LIST(l) ADC_BAT_STATES(l) l(UPDATE)
enum {
    ADC_BAT_STATES(ADC_EVENT_ENUM) ADC_EVENT_UPDATE
};

#define adc_event_strings adc_battery_states_str

#ifdef __cplusplus
}
#endif

#endif /* E49D2157_B2DD_488F_BD1E_79BD2A2D925F */
