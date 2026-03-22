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

    int32_t voltage = manager.getBusVoltage_mV();
    printf("Bus voltage: %li mV\n", voltage);

    ESP_LOGI(TAG, "DRV8323S SPI demo - starting");
 
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


    /* Periodic fault monitoring loop */
    ESP_LOGI(TAG, "Entering monitoring loop...");
    while (true) {
        float angle_deg = 0.0f;
        err = as5048a_read_angle_deg(enc, &angle_deg);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Angle: %.3f°", angle_deg);
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGI(TAG, "Angle: %.3f deg  (clearing stale EF)", angle_deg);
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}