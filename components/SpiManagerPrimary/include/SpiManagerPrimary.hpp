
#pragma once
#include "driver/gpio.h"

#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"
#include "AS5048.hpp"
#include "AS5048_Registers.hpp"

struct SpiConfigPrimary {
    gpio_num_t MOSI;
    gpio_num_t MISO;
    gpio_num_t CLK;
    
    gpio_num_t CS_MotorDriver;
};

class SpiManagerPrimary {
public:
    SpiManagerPrimary() = default;
    void begin(SpiConfigPrimary config);

    AS5048 encoder;
    DRV8323 motorDriver;
};
