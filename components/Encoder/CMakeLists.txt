idf_component_register(
    SRCS "Encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer driver BaseSPI
)
