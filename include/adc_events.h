#ifndef E49D2157_B2DD_488F_BD1E_79BD2A2D925F
#define E49D2157_B2DD_488F_BD1E_79BD2A2D925F

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declare an event base
ESP_EVENT_DECLARE_BASE(ADC_EVENT);        // declaration of the ADC_EVENT family

// declaration of the specific events under the ADC_EVENT family
enum {                                       
    ADC_EVENT_BATTERY_LOW,                  // battery level is low
    ADC_EVENT_BATTERY_CRITICAL,             // battery level is critical
    ADC_EVENT_BATTERY_OK,                   // battery level is ok
    ADC_EVENT_VOLTAGE_UPDATE,               // voltage is updated
};

#ifdef __cplusplus
}
#endif

#endif /* E49D2157_B2DD_488F_BD1E_79BD2A2D925F */
