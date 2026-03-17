
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

struct drvConfig {
    spi_host_device_t  spiHost;
    gpio_num_t         mosi;
    gpio_num_t         miso;
    gpio_num_t         sclk;
    gpio_num_t         cs;
    int                spiClockHz;
};

class DRV8323 {
    DRV8323(&config);
}
