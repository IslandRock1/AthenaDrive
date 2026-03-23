idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES DRV8323 esp_timer I2CManager ContinuousADC PID Encoder SpiManager
    )
