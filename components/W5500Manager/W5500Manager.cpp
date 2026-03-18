#include "W5500Manager.hpp"

#include "utility"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth_driver.h"
#include "esp_netif_types.h"
#include "driver/spi_master.h"
#include "esp_eth.h"

static const char* TAG = "W5500Manager";

// ---------------------------------------------------------------------------
// Event handler — just logs link up/down and DHCP assignment for now.
// Attach your own handler via esp_event_handler_register() if needed.
// ---------------------------------------------------------------------------
static void eth_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data)
{
    if (base == ETH_EVENT) {
        esp_eth_handle_t eth = *reinterpret_cast<esp_eth_handle_t*>(data);
        switch (id) {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(TAG, "Ethernet link up  [handle %p]", eth);
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "Ethernet link down [handle %p]", eth);
                break;
            case ETHERNET_EVENT_START:
                ESP_LOGI(TAG, "Ethernet started   [handle %p]", eth);
                break;
            case ETHERNET_EVENT_STOP:
                ESP_LOGI(TAG, "Ethernet stopped   [handle %p]", eth);
                break;
            default:
                break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t* event = reinterpret_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
W5500Manager::W5500Manager(gpio_num_t cs_master, gpio_num_t int_master,
                             gpio_num_t cs_slave,  gpio_num_t int_slave,
                             int spi_clock_mhz)
{
    // 1. Default event loop (needed for ETH_EVENT / IP_EVENT).
    //    Returns ESP_ERR_INVALID_STATE if already created — that's fine.
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return;
    }

    // 2. Initialize esp_netif.
    //    esp_netif_init() is idempotent in esp-idf 5.x.
    esp_netif_init();

    // 3. Register shared event handlers.
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,  &eth_event_handler, nullptr);
    esp_event_handler_register(IP_EVENT,  IP_EVENT_ETH_GOT_IP, &eth_event_handler, nullptr);

    // 4. Bring up both interfaces.
    ret = initOne(cs_master, int_master, spi_clock_mhz,
                  "ETH_MASTER", &_netif_master, &_eth_master);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init master W5500");
        return;
    }

    ret = initOne(cs_slave, int_slave, spi_clock_mhz,
                  "ETH_SLAVE", &_netif_slave, &_eth_slave);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init slave W5500");
        return;
    }

    _ok = true;
}

// ---------------------------------------------------------------------------
// Destructor — stop and destroy both interfaces
// ---------------------------------------------------------------------------
W5500Manager::~W5500Manager()
{
    for (auto [eth, netif] : {
            std::pair{_eth_master, _netif_master},
            std::pair{_eth_slave,  _netif_slave}})
    {
        if (eth) {
            esp_eth_stop(eth);
            esp_eth_del_netif_glue(
                reinterpret_cast<esp_eth_netif_glue_handle_t>(
                    esp_netif_get_io_driver(netif)));
            esp_eth_driver_uninstall(eth);
        }
        if (netif) {
            esp_netif_destroy(netif);
        }
    }
}

// ---------------------------------------------------------------------------
// initOne — configure and start a single W5500
// ---------------------------------------------------------------------------
esp_err_t W5500Manager::initOne(gpio_num_t cs, gpio_num_t intr, int clock_mhz,
                                 const char* key,
                                 esp_netif_t** out_netif,
                                 esp_eth_handle_t* out_eth)
{
    // --- netif ---------------------------------------------------------
    // Use the built-in ETH netif base config and override the key/hostname
    // so both interfaces are distinguishable.
    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base_cfg.if_key        = key;      // e.g. "ETH_MASTER"
    base_cfg.if_desc       = key;
    base_cfg.route_prio    = 20;       // lower than WiFi (100) by convention

    esp_netif_config_t netif_cfg = {
        .base   = &base_cfg,
        .driver = nullptr,
        .stack  = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };

    esp_netif_t* netif = esp_netif_new(&netif_cfg);
    if (!netif) {
        ESP_LOGE(TAG, "[%s] esp_netif_new failed", key);
        return ESP_FAIL;
    }

    // --- SPI device ----------------------------------------------------
    // SPI2_HOST bus is already initialised by SPIManagerSecondary.
    spi_device_interface_config_t spi_devcfg = {};
    spi_devcfg.command_bits     = 16;   // W5500 SPI frame: 16-bit address/cmd
    spi_devcfg.address_bits     = 8;    // + 8-bit control byte
    spi_devcfg.mode             = 0;
    spi_devcfg.clock_speed_hz   = clock_mhz * 1000 * 1000;
    spi_devcfg.spics_io_num     = cs;
    spi_devcfg.queue_size       = 20;

    spi_device_handle_t spi_handle = nullptr;
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &spi_devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] spi_bus_add_device failed: %s", key, esp_err_to_name(ret));
        esp_netif_destroy(netif);
        return ret;
    }

    // --- MAC -----------------------------------------------------------
    eth_w5500_config_t w5500_cfg = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    w5500_cfg.int_gpio_num = intr;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    if (!mac) {
        ESP_LOGE(TAG, "[%s] esp_eth_mac_new_w5500 failed", key);
        spi_bus_remove_device(spi_handle);
        esp_netif_destroy(netif);
        return ESP_FAIL;
    }

    // --- PHY -----------------------------------------------------------
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.autonego_timeout_ms = 0;   // W5500 internal PHY — no autoneg timeout
    phy_cfg.reset_gpio_num      = -1;  // no separate reset pin assumed
    esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&phy_cfg);
    if (!phy) {
        ESP_LOGE(TAG, "[%s] esp_eth_phy_new_w5500 failed", key);
        mac->del(mac);
        spi_bus_remove_device(spi_handle);
        esp_netif_destroy(netif);
        return ESP_FAIL;
    }

    // --- Driver install ------------------------------------------------
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = nullptr;
    ret = esp_eth_driver_install(&eth_cfg, &eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] esp_eth_driver_install failed: %s", key, esp_err_to_name(ret));
        phy->del(phy);
        mac->del(mac);
        spi_bus_remove_device(spi_handle);
        esp_netif_destroy(netif);
        return ret;
    }

    // --- Glue netif to driver ------------------------------------------
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    esp_netif_attach(netif, glue);

    // --- Start ---------------------------------------------------------
    ret = esp_eth_start(eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] esp_eth_start failed: %s", key, esp_err_to_name(ret));
        esp_eth_driver_uninstall(eth_handle);
        phy->del(phy);
        mac->del(mac);
        spi_bus_remove_device(spi_handle);
        esp_netif_destroy(netif);
        return ret;
    }

    *out_netif = netif;
    *out_eth   = eth_handle;
    ESP_LOGI(TAG, "[%s] W5500 started on CS=%d INT=%d", key, cs, intr);
    return ESP_OK;
}