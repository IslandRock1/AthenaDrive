#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "I2CManager.hpp"

#include "Pinout.hpp"
#include "Encoder.hpp"

extern "C" void app_main(void)
{
    // Deaktiver alle andre SPI-enheter
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    gpio_set_direction(CHIP_SELECT_MOTOR_DRIVER, GPIO_MODE_OUTPUT);
    gpio_set_level(CHIP_SELECT_MOTOR_DRIVER, 1);

    // Init SPI-bussen
    spi_bus_config_t busCfg = {
        .mosi_io_num   = SPI_MOSI_0,
        .miso_io_num   = SPI_MISO_0,
        .sclk_io_num   = SPI_CLK_0,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO));

    // Legg til encoder som SPI-device
    spi_device_interface_config_t devCfg = {};
    devCfg.clock_speed_hz = 1 * 1000 * 1000;
    devCfg.mode = 1;
    devCfg.spics_io_num = CHIP_SELECT_ENCODER;
    devCfg.queue_size = 1;
    devCfg.flags = 0;   // FULL DUPLEX

    spi_device_handle_t encoderHandle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devCfg, &encoderHandle));

    // Lag encoder-objekt
    Encoder enc(encoderHandle);

    if (enc.begin() != ESP_OK) {
        ESP_LOGE("MAIN", "Encoder not responding");
    } else {
        ESP_LOGI("MAIN", "Encoder initialized OK");
    }

    // Test-loop
    Encoder::Diagnostics diag;

    while (true) {
        if (enc.update() == ESP_OK) {
            uint16_t angle = enc.getRawAngle();
            enc.getDiagnostics(diag);

            ESP_LOGI("ANGLE",
                     "Angle=%u, AGC=%d, err=%d, parity=%d",
                     angle,
                     diag.agc,
                     diag.error_flag ? 1 : 0,
                     diag.parity_error ? 1 : 0);
        } else {
            ESP_LOGW("ANGLE", "Failed to read angle");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
