
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

struct SpiConfig {
    spi_host_device_t spiHost;
    gpio_num_t cs;
    int spiClockHz;
    uint8_t mode;
};

class BaseSPI {
public:
    BaseSPI() = default;
    ~BaseSPI();

    esp_err_t begin(SpiConfig config);
    virtual esp_err_t readRegister(uint16_t address, uint16_t &data) = 0;
    virtual esp_err_t writeRegister(uint16_t address, uint16_t data) = 0;
    virtual esp_err_t modifyBits(uint16_t address, uint16_t mask, uint16_t value) = 0;

protected:
    spi_device_handle_t _spiDevice = nullptr;
    spi_host_device_t _spiHost;

    esp_err_t _spiTransfer16(uint16_t tx, uint16_t &rx);
};