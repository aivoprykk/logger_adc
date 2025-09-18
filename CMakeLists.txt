
idf_component_register(
    SRCS adc.c
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_event
    PRIV_REQUIRES esp_adc esp_timer logger_common driver soc ulp
)

#
# ULP support additions to component CMakeLists.txt.
#
if(CONFIG_ULP_COPROC_ENABLED)
    message(STATUS "ULP coprocessor enabled, building ULP binary for logger_adc")
    # set(ULP_LD_FILE "ulp/ulp_memory.ld")  # Use default ESP-IDF ULP linker script
    # 1. The ULP app name must be unique (if multiple components use ULP).
    set(ulp_app_name "ulp_logger_adc")
    #
    # 2. Specify all assembly source files.
    #    Files should be placed into a separate directory (in this case, ulp/),
    #    which should not be added to COMPONENT_SRCS.
    set(ulp_s_sources "ulp/adc.S")
    #
    # 3. List all the component source files which include automatically
    #    generated ULP export file, ${ulp_app_name}.h:
    set(ulp_exp_dep_srcs "adc.c")
    #
    # 4. Call function to build ULP binary and embed in project using the argument
    #    values above.
    ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
else()
    message(STATUS "ULP coprocessor not enabled, skipping ULP build for logger_adc")
endif()
