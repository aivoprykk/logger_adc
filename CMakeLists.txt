
idf_component_register(
    SRCS adc.c adc_ulp.c
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_event logger_common
    PRIV_REQUIRES driver esp_adc soc ulp
)

#
# ULP support additions to component CMakeLists.txt.
#
if(CONFIG_ULP_COPROC_ENABLED)
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
else()
    message(STATUS "ULP coprocessor not enabled, skipping ULP build for logger_adc")
endif()
