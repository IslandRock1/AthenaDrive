#pragma once

#include <cstdint>
#include "driver/usb_serial_jtag.h"

// --- Shared data structures (must match PC side exactly) ---
#pragma pack(push, 1)
struct SensorData {
    uint32_t iteration;
    uint32_t timestamp_ms;
    float position;
    float velocity;
    float torque;
    int32_t current;
};

struct Command {
    uint8_t command_type;
    int32_t value0;
    float value1;
};
#pragma pack(pop)

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