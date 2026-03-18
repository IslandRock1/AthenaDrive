idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver esp_timer I2CManager esp_eth esp_netif esp_event nvs_flash
    )
