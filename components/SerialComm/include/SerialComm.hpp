#pragma once

#include <cstdint>
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "SerialConfig.hpp"

class SerialCom {
public:
    explicit SerialCom(size_t tx_buf_size = 1024, size_t rx_buf_size = 1024);

    void setData(const SensorData &data);
    bool getData(Command &cmd);
    void update();

private:
    static constexpr uint8_t SYNC_BYTE_0 = 0xAA;
    static constexpr uint8_t SYNC_BYTE_1 = 0x55;
    static constexpr int     RX_TIMEOUT_MS = 10;

    SensorData m_tx_data{};
};