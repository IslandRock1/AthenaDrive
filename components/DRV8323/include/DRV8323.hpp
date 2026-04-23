
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "BaseSPI.hpp"

struct MotorDriverConfig : public SpiConfig {
    gpio_num_t LOW_A;
    gpio_num_t LOW_B;
    gpio_num_t LOW_C;

    MotorDriverConfig(
        spi_host_device_t spiHost, 
        gpio_num_t cs, 
        int spiClockHz, 
        uint8_t mode, 
        gpio_num_t LOW_A, 
        gpio_num_t LOW_B, 
        gpio_num_t LOW_C)
        : SpiConfig(spiHost, cs, spiClockHz, mode), LOW_A(LOW_A), LOW_B(LOW_B), LOW_C(LOW_C) {}
};

class DRV8323 : public BaseSPI<MotorDriverConfig> {
public:
    esp_err_t begin(MotorDriverConfig config);
    void enable();
    esp_err_t readRegister(uint16_t address, uint16_t &data) override;
    esp_err_t writeRegister(uint16_t address, uint16_t data) override;

private:
    gpio_num_t LOW_A;
    gpio_num_t LOW_B;
    gpio_num_t LOW_C;
};