idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver esp_timer I2CManager SPIManagerSecondary PID SDCard DRV8323 AS5048 MCPWM
    )

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=missing-field-initializers)
