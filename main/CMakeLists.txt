idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." 
    REQUIRES driver log esp_timer I2CManager ContinuousADC PID Encoder SpiManager
    )
