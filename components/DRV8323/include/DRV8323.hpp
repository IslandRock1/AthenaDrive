
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

struct drvConfig {
    spi_host_device_t  spiHost;
    gpio_num_t         cs;
    int                spiClockHz;
};

class DRV8323 {
public:
    DRV8323(drvConfig &config);
    ~DRV8323();

    esp_err_t readRegister(uint8_t address, uint16_t *data);
    esp_err_t writeRegister(uint8_t address, uint16_t data);
    esp_err_t modifyBits(uint8_t address, uint16_t mask, uint16_t value);

private:
    spi_device_handle_t _spiDevice = nullptr;
    spi_host_device_t _spiHost;

    esp_err_t spiTransfer16(uint16_t tx, uint16_t *rx);
};