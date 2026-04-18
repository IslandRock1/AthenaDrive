#pragma once

#include <cstdint>
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "SerialConfig.hpp"

class SerialCom {
public:
    // Installs the USB Serial JTAG driver on construction.
    // tx_buf_size / rx_buf_size: internal driver ring-buffer sizes (bytes).
    explicit SerialCom(size_t tx_buf_size = 1024, size_t rx_buf_size = 1024);

    // Store the data that update() will transmit next.
    void setData(const SensorData &data);

    // Non-blocking attempt to read one Command from the host.
    // Returns true and fills `cmd` only if a full struct was received.
    bool getData(Command &cmd);

    // Transmit the most-recently set SensorData (with sync header).
    // Call this at whatever cadence you prefer from your task/loop.
    void update();

private:
    static constexpr uint8_t SYNC_BYTE_0 = 0xAA;
    static constexpr uint8_t SYNC_BYTE_1 = 0x55;
    static constexpr int     RX_TIMEOUT_MS = 10;

    SensorData m_tx_data{};
};