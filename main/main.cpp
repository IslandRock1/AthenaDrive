#include "drv8323.h"
#include "drv8323_regs.h"
#include "as5048a.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

#include "I2CManager.hpp"

#define PIN_MOSI    GPIO_NUM_47
#define PIN_MISO    GPIO_NUM_21
#define PIN_SCLK    GPIO_NUM_14
#define PIN_DRV_CS  GPIO_NUM_48
#define PIN_ENC_CS  GPIO_NUM_13
#define PIN_ENABLE  GPIO_NUM_NC
 
#define MOTOR_HIGH_A GPIO_NUM_1
#define MOTOR_LOW_A GPIO_NUM_42
#define MOTOR_HIGH_B GPIO_NUM_41
#define MOTOR_LOW_B GPIO_NUM_40
#define MOTOR_HIGH_C GPIO_NUM_39
#define MOTOR_LOW_C GPIO_NUM_38

/* SPI clock: datasheet max is ~10 MHz; 1 MHz is safe for bring-up */
#define SPI_CLK_HZ  1000000

void setupMotorPhasePins() {
    gpio_set_direction(MOTOR_HIGH_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LOW_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_HIGH_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LOW_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_HIGH_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LOW_C, GPIO_MODE_OUTPUT);
}

void turnOffAll() {
    gpio_set_level(MOTOR_HIGH_A, false);
    gpio_set_level(MOTOR_LOW_A, false);
    gpio_set_level(MOTOR_HIGH_B, false);
    gpio_set_level(MOTOR_LOW_B, false);
    gpio_set_level(MOTOR_HIGH_C, false);
    gpio_set_level(MOTOR_LOW_C, false);
}

extern "C" void app_main(void)
{

    I2CManager manager{GPIO_NUM_11, GPIO_NUM_12};
    manager.writePin(11, true); // Enable Motor
    manager.writePin(12, true); // Auto cal motor

    for (int i = 0; i < 20; i++) {
        int32_t voltage = manager.getBusVoltage_mV();
        printf("Bus voltage: %li mV. Starting in %i seconds.\n", voltage, 20 - i);
        manager.writePin(14, i % 2 == 0); // Led
        manager.writePin(15, i % 2 != 0); // Led

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
 
    /* Build configuration */
    drv8323_config_t cfg = {
        .spi_host     = SPI2_HOST,
        .mosi         = PIN_MOSI,
        .miso         = PIN_MISO,
        .sclk         = PIN_SCLK,
        .cs           = PIN_DRV_CS,
        .enable       = PIN_ENABLE,
        .spi_clock_hz = SPI_CLK_HZ,
    };
 
    drv8323_handle_t dev = NULL;
    esp_err_t err = drv8323_init(&cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drv8323_init failed: %s", esp_err_to_name(err));
        return;
    }
 
    as5048a_handle_t enc = NULL;
    err = as5048a_init(SPI2_HOST, PIN_ENC_CS, SPI_CLK_HZ, &enc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "as5048a_init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Enable the DRV (pull ENABLE high) */
    drv8323_enable(dev, true);
    vTaskDelay(pdMS_TO_TICKS(10));
    drv8323_clear_faults(dev);
    drv8323_print_faults(dev);
 
    /* Dump all registers before configuration */
    ESP_LOGI(TAG, "--- Before configuration ---");
    dump_all_registers(dev);
 
    /* Apply minimal configuration */
    err = configure_drv(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configure_drv failed: %s", esp_err_to_name(err));
    }
 
    /* Dump all registers after configuration */
    ESP_LOGI(TAG, "--- After configuration ---");
    dump_all_registers(dev);

    ESP_LOGI(TAG, "AS5048A encoder - starting");
 
    /* Give the encoder a moment after power-up (startup time ≤10 ms) */
    vTaskDelay(pdMS_TO_TICKS(15));
 
    /* Clear any spurious error flags from startup */
    as5048a_clear_error(enc);
 
    /* Check diagnostics – confirms magnet is present and field is in range */
    as5048a_print_diagnostics(enc);

    setupMotorPhasePins();
    turnOffAll();

    int iteration = 0;
    int phaseIx = 0;
    ESP_LOGI(TAG, "Entering monitoring loop...");
    while (true) {
        iteration++;

        float angle_deg = 0.0f;
        err = as5048a_read_angle_deg(enc, &angle_deg);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Angle: %.3f°", angle_deg);
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            // ESP_LOGI(TAG, "Angle: %.3f deg  (clearing stale EF)", angle_deg);
            as5048a_clear_error(enc);
        } else {
            ESP_LOGE(TAG, "Encoder SPI error: %s", esp_err_to_name(err));
        }

        uint16_t fs1 = 0;
        drv8323_read_reg(dev, DRV_REG_FAULT_STATUS1, &fs1);
        if (fs1 & DRV_FS1_FAULT) {
            ESP_LOGW(TAG, "FAULT detected! fs1=0x%03X", fs1);
            drv8323_print_faults(dev);
            drv8323_clear_faults(dev);
        }

        // x Hz
        if (iteration % 20 == 0) {
            turnOffAll();
            phaseIx++;
            if (phaseIx == 6) { phaseIx = 0; }
            printf("Phase ix: %i.\n", phaseIx);
            
            // Should probably use switch case.
            if (phaseIx == 0) {
                gpio_set_level(MOTOR_HIGH_A, true);
                gpio_set_level(MOTOR_LOW_B, true);
            } else if (phaseIx == 1) {
                gpio_set_level(MOTOR_HIGH_A, true);
                gpio_set_level(MOTOR_LOW_C, true);
            } else if (phaseIx == 2) {
                gpio_set_level(MOTOR_HIGH_B, true);
                gpio_set_level(MOTOR_LOW_C, true);
            } else if (phaseIx == 3) {
                gpio_set_level(MOTOR_HIGH_B, true);
                gpio_set_level(MOTOR_LOW_A, true);
            } else if (phaseIx == 4) {
                gpio_set_level(MOTOR_HIGH_C, true);
                gpio_set_level(MOTOR_LOW_A, true);
            } else if (phaseIx == 5) {
                gpio_set_level(MOTOR_HIGH_C, true);
                gpio_set_level(MOTOR_LOW_B, true);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}