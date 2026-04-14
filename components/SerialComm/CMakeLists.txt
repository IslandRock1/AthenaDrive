idf_component_register(
    SRCS "SerialComm.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_usb_serial_jtag
)