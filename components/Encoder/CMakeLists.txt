idf_component_register(
    SRCS "Encoder.cpp"
    INCLUDE_DIRS "." "include"
    REQUIRES driver log SpiManager esp_timer
)
