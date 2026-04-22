
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "BaseSPI.hpp"

struct MotorDriverConfig : public SpiConfig {
    gpio_num_t A_LOW;
    gpio_num_t B_LOW;
    gpio_num_t C_LOW;
}

class DRV8323 : public BaseSPI {
public:
    esp_err_t readRegister(uint16_t address, uint16_t &data) override;
    esp_err_t writeRegister(uint16_t address, uint16_t data) override;
    esp_err_t modifyBits(uint16_t address, uint16_t mask, uint16_t value) override;
};