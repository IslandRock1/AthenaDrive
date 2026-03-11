
#pragma once

#include "memory"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"

#include "SDCard.hpp"

class SPIManagerSecondary {
public:
    SPIManagerSecondary(gpio_num_t MOSI, gpio_num_t MISO, gpio_num_t CLK, gpio_num_t CS_SDCard);
};