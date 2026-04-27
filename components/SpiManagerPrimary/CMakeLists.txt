idf_component_register(
    SRCS "SpiManagerPrimary.cpp"
    INCLUDE_DIRS "include"
    REQUIRES Encoder MotorDriver driver
)
