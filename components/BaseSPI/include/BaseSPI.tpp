
#pragma once

template<typename ConfigT>
BaseSPI<ConfigT>::~BaseSPI() {
    if (_spiDevice) {
        spi_bus_remove_device(_spiDevice);
    }
}

template<typename ConfigT>
esp_err_t BaseSPI<ConfigT>::begin(ConfigT config) {
    _spiHost = config.spiHost;

    spi_device_interface_config_t devConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = config.mode,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = config.spiClockHz,
        .spics_io_num = config.cs,
        .queue_size = 1,
    };

    return spi_bus_add_device(config.spiHost, &devConfig, &_spiDevice);
}

template<typename ConfigT>
esp_err_t BaseSPI<ConfigT>::_spiTransfer16(uint16_t tx, uint16_t &rx) {
    uint8_t txBuffer[2] = {
        static_cast<uint8_t>((tx >> 8) & 0xFF),
        static_cast<uint8_t>(tx & 0xFF)
    };
    uint8_t rxBuffer[2] = {0, 0};

    spi_transaction_t spiTransaction = {
        .length = 16,
        .tx_buffer = txBuffer,
        .rx_buffer = rxBuffer,
    };

    esp_err_t err = spi_device_transmit(_spiDevice, &spiTransaction);
    rx = (rxBuffer[0] << 8) | rxBuffer[1];

    return err;
}

template<typename ConfigT>
esp_err_t BaseSPI<ConfigT>::modifyBits(uint16_t address, uint16_t mask, uint16_t value) {
    uint16_t current = 0;
    esp_err_t err = readRegister(address, current);
    if (err != ESP_OK) return err;

    uint16_t newValue = (current & ~mask) | (value & mask);
    return writeRegister(address, newValue);
}