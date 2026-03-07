#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

class Multiplexer
{
public:
    Multiplexer();
    void begin(i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed);

    esp_err_t setPortMode(uint16_t mode);      // Set direction (1=input, 0=output)
    esp_err_t writePort(uint16_t value);       // Write full 16-bit port
    esp_err_t writePin(uint8_t pin, bool level); // Write single pin
    esp_err_t readPort(uint16_t &value);       // Read full 16-bit port
    esp_err_t readPin(uint8_t pin, bool &value); // Read single pin

private:
    i2c_master_dev_handle_t dev;

    esp_err_t writeRegister(uint8_t reg, uint16_t value);
    esp_err_t readRegister(uint8_t reg, uint16_t &value);
};