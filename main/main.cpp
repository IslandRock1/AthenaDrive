<<<<<<< HEAD
=======
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "SDCard.hpp"
#include "SPIManagerSecondary.hpp"

extern "C" void app_main(void)
{
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    bool state = true;

    SPIManagerSecondary spiManager{
        SPI_MOSI_1, SPI_MISO_1, SPI_CLK_1, CHIP_SELECT_SD
    };

    SDCard::writeFile("Test39", "Hello Oscar: %i.\n", state);
    SDCard::writeFile("Test39", "Here i have appended something to the end.\n");

    int32_t iteration = 0;
    int64_t startTime = esp_timer_get_time();
    while (1) {
        iteration++;

        int64_t timeNow = esp_timer_get_time();
        if (timeNow - startTime > 1000000) {

            state = !state;
            i2cManager.writePin(MULTIPLEXER_LED0, state);
            i2cManager.writePin(MULTIPLEXER_LED1, !state);

            startTime = timeNow;
        }
        
    }
}
>>>>>>> main
