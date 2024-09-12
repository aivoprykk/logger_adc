
idf_component_register(
    SRCS adc.c
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_event
    PRIV_REQUIRES esp_adc esp_timer logger_common
)
