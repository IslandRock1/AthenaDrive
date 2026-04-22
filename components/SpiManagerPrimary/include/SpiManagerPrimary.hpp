
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
    spi_host_device_t SPI_HOST;
};

class SpiManagerPrimary {
public:
    SpiManagerPrimary() = default;
    void beginManager(SpiConfigPrimary config);
    void beginEncoder(EncoderConfig config);
    void beginMotorDriver(MotorDriverConfig config);

    AS5048 encoder;
    DRV8323 motorDriver;
};
