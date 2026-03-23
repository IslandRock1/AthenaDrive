
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

struct encoderConfig {
    spi_host_device_t  spiHost;
    gpio_num_t         cs;
    int                spiClockHz;
};

class AS5048 {
public:
    AS5048(encoderConfig &config);
    ~AS5048();

    esp_err_t readRegister(uint16_t address, uint16_t *data);
    esp_err_t writeRegister(uint16_t address, uint16_t data);
    esp_err_t modifyBits(uint16_t address, uint16_t mask, uint16_t value);

private:
    spi_device_handle_t _spiDevice = nullptr;
    spi_host_device_t _spiHost;

    esp_err_t spiTransfer16(uint16_t tx, uint16_t *rx);
};