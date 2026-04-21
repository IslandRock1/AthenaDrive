#include <stdio.h>
#include "BaseSPI.hpp"

BaseSPI::~BaseSPI() {
    if (_spiDevice) {
        spi_bus_remove_device(_spiDevice);
    }
}

esp_err_t BaseSPI::begin(SpiConfig config) {
    _spiHost = spi_host_device_t(config.spiHost);

    spi_device_interface_config_t devConfig = {
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
        .mode           = config.mode,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = config.spiClockHz,
        .spics_io_num   = config.cs,
        .queue_size     = 1,
    };

    return spi_bus_add_device(config.spiHost, &devConfig, &_spiDevice);
}

esp_err_t BaseSPI::_spiTransfer16(uint16_t tx, uint16_t &rx) {
    uint8_t txBuffer[2] = { static_cast<uint8_t>((tx >> 8) & 0xFF), static_cast<uint8_t>(tx & 0xFF) };
    uint8_t rxBuffer[2] = { 0, 0 };

    spi_transaction_t spiTransaction = {
        .length = 16,
        .tx_buffer = txBuffer,
        .rx_buffer = rxBuffer,
    };

    esp_err_t err = spi_device_transmit(_spiDevice, &spiTransaction);

    if (rx) {
        rx = ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];
    }

    return err;
}