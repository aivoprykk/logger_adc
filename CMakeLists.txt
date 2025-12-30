set(srcs adc_base.c adc_battery_adp.c adc_snapshot.c)
if(CONFIG_LOGGER_ADC_MODE_ONESHOT)
    list(APPEND srcs adc_oneshot.c)
endif()

if(CONFIG_ULP_COPROC_ENABLED)
set(ulpsrcs adc_ulp.c ulp_hardware.c ulp_program.c)
endif()

idf_component_register(
    SRCS ${srcs} ${ulpsrcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_event logger_common main
    PRIV_REQUIRES driver esp_adc
)

#
# ULP support additions to component CMakeLists.txt.
#
if(CONFIG_ULP_COPROC_ENABLED)
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::soc)
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::ulp)
    message(STATUS "ULP coprocessor enabled, building ULP binary for logger_adc")
    # 1. The ULP app name must be unique (if multiple components use ULP).
    set(ulp_app_name "ulp_battery")
    # 2. Specify all assembly source files.
    set(ulp_s_sources "ulp/adc.S")
    # 3. List all the component source files which include automatically
    #    generated ULP export file, ${ulp_app_name}.h:
    set(ulp_exp_dep_srcs "adc_ulp.c")
    # 4. Call function to build ULP binary and embed in project using the argument
    #    values above.
    ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
    # 5. Add compile definitions for ULP
else()
    message(STATUS "ULP coprocessor not enabled, skipping ULP build for logger_adc")
endif()
