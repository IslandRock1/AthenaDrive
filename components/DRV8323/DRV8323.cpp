#include <stdio.h>
#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"

DRV8323::DRV8323(drvConfig &config)
    : _spiHost(config.spiHost) {
    
    spi_device_interface_config_t devConfig = {
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
        .mode           = 1,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = config.spiClockHz > 0 ? config.spiClockHz : 1000000,
        .spics_io_num   = config.cs,
        .queue_size     = 1,
    };

    esp_err_t err = spi_bus_add_device(config.spiHost, &devConfig, &_spiDevice);
    // TODO: Error handling.
}

DRV8323::~DRV8323() {
    if (_spiDevice) {
        spi_bus_remove_device(_spiDevice);
    }
}

esp_err_t DRV8323::readRegister(uint8_t address, uint16_t *data) {
    uint16_t tx = DRV_SPI_READ | DRV_SPI_ADDR(address) | 0x0000;
    esp_err_t err = spiTransfer16(tx, data);
    return err;
}

esp_err_t DRV8323::writeRegister(uint8_t address, uint16_t data) {
    uint16_t tx = DRV_SPI_WRITE | DRV_SPI_ADDR(address) | DRV_SPI_DATA(data);
    uint16_t rx = 0;
    esp_err_t err = spiTransfer16(tx, &rx);
    return err;
}

esp_err_t DRV8323::modifyBits(uint8_t address, uint16_t mask, uint16_t value) {
    uint16_t current = 0;
    esp_err_t err = readRegister(address, &current);
    if (err != ESP_OK) { return err; }

    uint16_t newValue = (current & ~mask) | (value & mask);
    err = writeRegister(address, newValue);
    return err;
}

esp_err_t DRV8323::spiTransfer16(uint16_t tx, uint16_t *rx) {
    uint8_t txBuffer[2] = { static_cast<uint8_t>((tx >> 8) & 0xFF), static_cast<uint8_t>(tx & 0xFF) };
    uint8_t rxBuffer[2] = { 0, 0 };

    spi_transaction_t spiTransaction = {
        .length = 16,
        .tx_buffer = txBuffer,
        .rx_buffer = rxBuffer,
    };

    esp_err_t err = spi_device_transmit(_spiDevice, &spiTransaction);

    if (rx) {
        *rx = ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];
        *rx &= DRV_SPI_DATA_MASK;
    }

    return err;
}
