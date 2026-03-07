
#pragma once

#include <memory>
#include "driver/gpio.h"

#include "Multiplexer.hpp"
#include "ina226_interface.h"

class I2CManager {
public:
    I2CManager(gpio_num_t sda, gpio_num_t scl);

    // Multiplexer
    bool readPin(int pin);
    void writePin(int pin, bool value);

    // INA226s
    int32_t getBusVoltage_mV();
    int32_t getCurrent_mA();

private:
    Multiplexer _multiplexer{};
    std::unique_ptr<INA226> _currentSensor;
};