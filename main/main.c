#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "w5500.h"

#include "Pinout.hpp"

static const char *TAG = "main";

// The IP you want to ping — change this to something on your network
static const uint8_t PING_TARGET[] = { 192, 168, 1, 131 };

void app_main(void)
{
    gpio_set_direction(CHIP_SELECT_SD, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_1, GPIO_MODE_OUTPUT);
    gpio_set_level(CHIP_SELECT_SD, 1);
    gpio_set_level(CHIP_SELECT_W5500_1, 1);
    gpio_set_pull_mode(ETHERNET_INT_0, GPIO_PULLUP_ONLY);
    gpio_install_isr_service(0);

    ESP_LOGI(TAG, "=== W5500 Init + Ping Test ===");

    // Initialise the W5500 (SPI bus, chip reset, MAC/IP config)
    if (w5500_init() != ESP_OK) {
        ESP_LOGE(TAG, "W5500 init failed — halting. Check wiring and power.");
        return;
    }

    // Wait for Ethernet link to come up
    ESP_LOGI(TAG, "Waiting for link...");
    for (int i = 0; i < 50; i++) {
        if (w5500_link_up()) {
            ESP_LOGI(TAG, "Link is UP");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        if (i == 49) {
            ESP_LOGW(TAG, "Link did not come up after 5 s — is the cable plugged in?");
        }
    }

    // Send 5 pings, one per second
    for (int i = 0; i < 5; i++) {
        esp_err_t ret = w5500_ping(PING_TARGET, 1000);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Ping error (not a timeout) — something is wrong");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Done.");
}
