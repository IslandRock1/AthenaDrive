#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"

extern "C" void app_main(void)
{
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    gpio_set_direction(CHIP_SELECT_ENCODER, GPIO_MODE_OUTPUT);
    gpio_set_level(CHIP_SELECT_ENCODER, 1);

    bool state = true;

    spi_bus_config_t busCfg = {
        .mosi_io_num   = SPI_MOSI_0,
        .miso_io_num   = SPI_MISO_0,
        .sclk_io_num   = SPI_CLK_0,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    drvConfig config = {
        .spiHost = SPI2_HOST,
        .cs = CHIP_SELECT_MOTOR_DRIVER,
        .spiClockHz = 100000,
    };
    DRV8323 motorDriver = DRV8323(config);

    int32_t iteration = 0;
    int64_t startTime = esp_timer_get_time();
    while (1) {
        iteration++;

        int64_t timeNow = esp_timer_get_time();
        if (timeNow - startTime > 1000000) {

            state = !state;
            i2cManager.writePin(MULTIPLEXER_LED0, state);
            i2cManager.writePin(MULTIPLEXER_LED1, !state);

            printf("########################\n");
            printf("Bus voltage: %li mV.\n", i2cManager.getBusVoltage_mV());

            for (int i = 0; i < 6; i++) {
                uint16_t data = 0;
                motorDriver.readRegister(i, &data);
                printf("Address: %i | Data: %i\n", i, data);
            }

            startTime = timeNow;
        }
        
    }
}
