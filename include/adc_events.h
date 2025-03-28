#ifndef E49D2157_B2DD_488F_BD1E_79BD2A2D925F
#define E49D2157_B2DD_488F_BD1E_79BD2A2D925F

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "logger_common.h"

// Declare an event base
ESP_EVENT_DECLARE_BASE(ADC_EVENT);        // declaration of the ADC_EVENT family

#define ADC_EVENT_LIST(l) \
    l(ADC_EVENT_BATTERY_LOW) \
    l(ADC_EVENT_BATTERY_CRITICAL) \
    l(ADC_EVENT_BATTERY_OK) \
    l(ADC_EVENT_UPDATE)

// declaration of the specific events under the UBX_EVENT family
enum {                                       
    ADC_EVENT_LIST(ENUM)
};

const char * adc_event_strings(int id);

#ifdef __cplusplus
}
#endif

#endif /* E49D2157_B2DD_488F_BD1E_79BD2A2D925F */
