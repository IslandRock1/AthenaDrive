idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver esp_timer I2CManager SPIManagerSecondary SDCard DRV8323
    )
