#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "I2CManager.hpp"

#define SDA GPIO_NUM_11
#define SCL GPIO_NUM_12

#define LED_GPIO GPIO_NUM_2

extern "C" void app_main()
{
    I2CManager i2cManager(SDA, SCL);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while(true)
    {

        gpio_set_level(LED_GPIO, 1);
        i2cManager.writePin(14, 1);
        // vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_GPIO, 0);
        i2cManager.writePin(14, 0);
        // vTaskDelay(pdMS_TO_TICKS(500));

        printf("Voltage: %li mV | Current: %li mA\n", i2cManager.getBusVoltage_mV(), i2cManager.getCurrent_mA());
    }
}