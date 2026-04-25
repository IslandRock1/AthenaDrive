#include "SerialComm.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "SerialCom";

SerialCom::SerialCom(size_t tx_buf_size, size_t rx_buf_size) {
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = tx_buf_size,
        .rx_buffer_size = rx_buf_size,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    ESP_LOGI(TAG, "USB Serial JTAG driver installed");
}

void SerialCom::setData(const SensorData &data) {
    m_tx_data = data;
}

bool SerialCom::getData(Command &cmd) {
    int bytes_read = usb_serial_jtag_read_bytes(
        reinterpret_cast<char *>(&cmd),
        sizeof(Command),
        pdMS_TO_TICKS(RX_TIMEOUT_MS)
    );
    return bytes_read == sizeof(Command);
}

void SerialCom::update() {
    uint8_t frame[2 + sizeof(SensorData)];
    frame[0] = SYNC_BYTE_0;
    frame[1] = SYNC_BYTE_1;
    memcpy(frame + 2, &m_tx_data, sizeof(SensorData));

    int bytes_sent = usb_serial_jtag_write_bytes(
        reinterpret_cast<const char *>(frame), sizeof(frame), portMAX_DELAY);
    usb_serial_jtag_ll_txfifo_flush();

    if ((bytes_sent - 2) != sizeof(SensorData)) {
        ESP_LOGE(TAG, "Failed to send full SensorData (sent %d)", bytes_sent);
    }
}