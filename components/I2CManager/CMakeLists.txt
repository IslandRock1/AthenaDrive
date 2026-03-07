idf_component_register(
    SRCS "I2CManager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver Multiplexer ina226
)
