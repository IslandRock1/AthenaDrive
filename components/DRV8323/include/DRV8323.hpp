
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "BaseSPI.hpp"

class DRV8323 : public BaseSPI {
public:
    esp_err_t readRegister(uint8_t address, uint16_t &data) override;
    esp_err_t writeRegister(uint8_t address, uint16_t data) override;
    esp_err_t modifyBits(uint8_t address, uint16_t mask, uint16_t value) override;
};