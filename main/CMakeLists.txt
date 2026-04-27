idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver esp_timer I2CManager SPIManagerSecondary PID SDCard MotorDriver Encoder MCPWM SerialComm Controller OneshotADC SpiManagerPrimary
    )

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=missing-field-initializers)
