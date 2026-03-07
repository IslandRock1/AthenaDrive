#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "ina226_interface.h"
#include "Multiplexer.hpp"

#define SDA GPIO_NUM_11
#define SCL GPIO_NUM_12

#define LED_GPIO GPIO_NUM_2

extern "C" void app_main()
{

    // -------------------------
    // Create I2C Bus
    // -------------------------

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };

    i2c_master_bus_handle_t bus;

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus)
    );

    // -------------------------
    // Devices
    // -------------------------

    Multiplexer mcp(bus, 0x20, 100000);

    INA226 currentSensor(bus, 0x40, 100000);

    currentSensor.InitDriver(3.5, 30.0);

    // MCP all outputs
    mcp.setPortMode(0x0000);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // -------------------------
    // Loop
    // -------------------------

    while(true)
    {

        gpio_set_level(LED_GPIO, 1);

        mcp.setPin(14, true);
        mcp.setPin(15, false);

        printf("Bus voltage: %ld mV\n",
            currentSensor.GetBusVoltage_mV());

        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_GPIO, 0);

        mcp.setPin(14, false);
        mcp.setPin(15, true);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}