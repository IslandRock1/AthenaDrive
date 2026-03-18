idf_component_register(
    SRCS "W5500Manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_eth esp_netif
)
