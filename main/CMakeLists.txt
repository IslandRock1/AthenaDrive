idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver log esp_timer I2CManager SPIManagerSecondary SDCard ContinuousADC PID Encoder SpiManager
    )
