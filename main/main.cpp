#include "drv8323.h"
#include "drv8323_regs.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

#include "I2CManager.hpp"

#define PIN_MOSI    GPIO_NUM_47
#define PIN_MISO    GPIO_NUM_21
#define PIN_SCLK    GPIO_NUM_14
#define PIN_CS      GPIO_NUM_48
#define PIN_ENABLE  GPIO_NUM_NC
 
/* SPI clock: datasheet max is ~10 MHz; 1 MHz is safe for bring-up */
#define SPI_CLK_HZ  1000000

static void dump_all_registers(drv8323_handle_t dev)
{
    static const char *reg_names[] = {
        "Fault Status 1 (R)",
        "VGS Status 2   (R)",
        "Driver Control  (RW)",
        "Gate Drive HS   (RW)",
        "Gate Drive LS   (RW)",
        "OCP Control     (RW)",
        "CSA Control     (RW)",
        "Reserved        (RW)",
    };
 
    ESP_LOGI(TAG, "====== DRV8323S Register Dump ======");
    for (uint8_t addr = 0; addr <= 0x07; addr++) {
        uint16_t val = 0;
        esp_err_t err = drv8323_read_reg(dev, addr, &val);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  [0x%02X] %-24s = 0x%03X  (%011ub)",
                     addr, reg_names[addr], val, val);
        } else {
            ESP_LOGE(TAG, "  [0x%02X] read error: %s", addr, esp_err_to_name(err));
        }
    }
    ESP_LOGI(TAG, "====================================");
}

static esp_err_t configure_drv(drv8323_handle_t dev)
{
    esp_err_t err;
 
    /* 1. Unlock SPI registers before writing (LOCK field in Gate Drive HS) */
    err = drv8323_rmw_reg(dev, DRV_REG_GATE_DRIVE_HS,
                           DRV_LOCK_MASK, DRV_LOCK_UNLOCK);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "SPI registers unlocked");
 
    /* 2. Set PWM mode to 6x (one INx pin per gate) – most common for FOC */
    err = drv8323_rmw_reg(dev, DRV_REG_GATE_DRIVE_HS,
                           DRV_PWM_MODE_MASK, DRV_PWM_MODE_6X);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "PWM mode set to 6x");
 
    /* 3. Set CSA gain to 20 V/V (DRV8323S only) */
    err = drv8323_rmw_reg(dev, DRV_REG_CSA_CTRL,
                           DRV_CSA_GAIN_MASK, DRV_CSA_GAIN_20);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "CSA gain set to 20 V/V");
 
    /* 4. Enable OTW reporting on nFAULT */
    err = drv8323_rmw_reg(dev, DRV_REG_DRIVER_CTRL,
                           DRV_DC_OTW_REP, DRV_DC_OTW_REP);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "OTW reporting enabled");
 
    return ESP_OK;
}

extern "C" void app_main(void)
{

    I2CManager manager{GPIO_NUM_11, GPIO_NUM_12};
    manager.writePin(11, true); // Enable Motor
    manager.writePin(12, true); // Auto cal motor
    manager.writePin(14, true); // Led
    manager.writePin(15, true); // Led

    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, true); // Chip select encoder HIGH

    int32_t voltage = manager.getBusVoltage_mV();
    printf("Bus voltage: %li mV\n", voltage);

    ESP_LOGI(TAG, "DRV8323S SPI demo – starting");
 
    /* Build configuration */
    drv8323_config_t cfg = {
        .spi_host     = SPI2_HOST,
        .mosi         = PIN_MOSI,
        .miso         = PIN_MISO,
        .sclk         = PIN_SCLK,
        .cs           = PIN_CS,
        .enable       = PIN_ENABLE,
        .spi_clock_hz = SPI_CLK_HZ,
    };
 
    drv8323_handle_t dev = NULL;
    esp_err_t err = drv8323_init(&cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drv8323_init failed: %s", esp_err_to_name(err));
        return;
    }
 
    /* Enable the DRV (pull ENABLE high) */
    drv8323_enable(dev, true);
 
    /* Wait for the DRV to come out of reset / charge pump to settle */
    vTaskDelay(pdMS_TO_TICKS(10));
 
    /* Clear any latched faults from power-up */
    drv8323_clear_faults(dev);
 
    /* Print initial fault state */
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
 
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_42, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_41, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_40, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_1, true);
    gpio_set_level(GPIO_NUM_42, false);
    gpio_set_level(GPIO_NUM_41, false);
    gpio_set_level(GPIO_NUM_40, false);
    gpio_set_level(GPIO_NUM_39, false);
    gpio_set_level(GPIO_NUM_38, true);


    /* Periodic fault monitoring loop */
    ESP_LOGI(TAG, "Entering monitoring loop (fault check every 1 s)...");
    while (true) {
        uint16_t fs1 = 0;
        drv8323_read_reg(dev, DRV_REG_FAULT_STATUS1, &fs1);
        if (fs1 & DRV_FS1_FAULT) {
            ESP_LOGW(TAG, "FAULT detected! fs1=0x%03X", fs1);
            drv8323_print_faults(dev);
            drv8323_clear_faults(dev);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}