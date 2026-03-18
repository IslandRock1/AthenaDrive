#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "esp_eth_driver.h"
#include "esp_eth_phy.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"

#define ETH_SPI_HOST      SPI2_HOST
#define ETH_SCLK_GPIO     SPI_CLK_1
#define ETH_MOSI_GPIO     SPI_MOSI_1
#define ETH_MISO_GPIO     SPI_MISO_1
#define ETH_CS_GPIO       CHIP_SELECT_W5500_0
#define ETH_INT_GPIO      ETHERNET_INT_0
#define ETH_RST_GPIO      GPIO_NUM_NC
#define ETH_SPI_CLOCK_MHZ 8

#define ETH_CONNECTED_BIT BIT0
static EventGroupHandle_t s_eth_event_group;

esp_netif_t *eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI("ETH", "Link Up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI("ETH", "Link Down");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI("ETH", "Started");
            break;
        default:
            break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    ESP_LOGI("IP_EVENT", "Event ID: %ld", event_id);
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("ETH", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
    }
}

void ethernet_init(void)
{
    // 1. Init TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    // 3. Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                &ip_event_handler, NULL));

    // 4. Init SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num   = ETH_MOSI_GPIO,
        .miso_io_num   = ETH_MISO_GPIO,
        .sclk_io_num   = ETH_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 5. W5500 MAC config (SPI interface)
    spi_device_interface_config_t spiConfig = {
        .command_bits    = 16,
        .address_bits    = 8,
        .mode            = 0,
        .clock_speed_hz  = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num    = ETH_CS_GPIO,
        .queue_size      = 20,
    };
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &spiConfig);
    w5500_config.int_gpio_num = ETH_INT_GPIO;

    // 6. MAC and PHY init
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = ETH_RST_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // 7. Install and start Ethernet driver
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    uint8_t mac_addr[6] = {0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56};
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

    // 8. Attach netif glue
    esp_eth_netif_glue_handle_t eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_glue));

    // 9. Start!
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    esp_netif_dhcpc_start(eth_netif);
}

extern "C" void app_main(void)
{
    bool state = true;
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_ETHERNET_RESET_0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    i2cManager.writePin(MULTIPLEXER_ETHERNET_RESET_0, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_direction(CHIP_SELECT_SD, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_1, GPIO_MODE_OUTPUT);
    gpio_set_level(CHIP_SELECT_SD, 1);
    gpio_set_level(CHIP_SELECT_W5500_1, 1);
    gpio_set_pull_mode(ETH_INT_GPIO, GPIO_PULLUP_ONLY);
    gpio_install_isr_service(0);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_eth_event_group = xEventGroupCreate();
    ethernet_init();

    int attempts = 0;
    while (attempts < 20) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
            ESP_LOGI("APP", "IP: " IPSTR, IP2STR(&ip_info.ip));
            if (ip_info.ip.addr != 0) break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        attempts++;
    }

    ESP_LOGI("APP", "Waiting for IP...");
    xEventGroupWaitBits(s_eth_event_group, ETH_CONNECTED_BIT,
        false, true, portMAX_DELAY);
    ESP_LOGI("APP", "Ready — starting app logic");

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
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}