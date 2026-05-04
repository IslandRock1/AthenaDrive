idf_component_register(
    SRCS "RealTimeTask.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer SpiManagerPrimary OneshotADC Controller PID GlobalVariableManager MCPWM LowpassFilter SerialComm
)
