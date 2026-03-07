#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

class Multiplexer
{
public:

    Multiplexer(i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed);

    esp_err_t setPortMode(uint16_t mode);
    esp_err_t writePort(uint16_t value);
    esp_err_t setPin(uint8_t pin, bool level);

private:

    i2c_master_dev_handle_t dev;

    esp_err_t writeRegister(uint8_t reg, uint16_t value);
    esp_err_t readRegister(uint8_t reg, uint16_t &value);

};