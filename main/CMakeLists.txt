idf_component_register(
    SRCS "main.c" "w5500.c"
    INCLUDE_DIRS "." "Include"
    REQUIRES driver esp_timer
    )
