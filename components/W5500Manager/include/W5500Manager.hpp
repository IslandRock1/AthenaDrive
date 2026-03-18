#pragma once

#include "esp_eth.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

class W5500Manager {
public:
    /**
     * @brief Construct and start both W5500 ethernet interfaces.
     *
     * The SPI bus (SPI2_HOST) must already be initialized by SPIManagerSecondary
     * before constructing this object.
     *
     * @param cs_master     CS GPIO for the master-facing W5500
     * @param int_master    INT GPIO for the master-facing W5500
     * @param cs_slave      CS GPIO for the slave-facing W5500
     * @param int_slave     INT GPIO for the slave-facing W5500
     * @param spi_clock_mhz SPI clock in MHz (W5500 supports up to 80 MHz, 20 is conservative)
     */
    W5500Manager(gpio_num_t cs_master, gpio_num_t int_master,
                 gpio_num_t cs_slave,  gpio_num_t int_slave,
                 int spi_clock_mhz = 20);

    ~W5500Manager();

    /** @brief Returns true if both interfaces came up without error. */
    bool isOk() const { return _ok; }

    /** @brief Netif handle for the master-facing interface (ETH_MASTER). */
    esp_netif_t* netifMaster() const { return _netif_master; }

    /** @brief Netif handle for the slave-facing interface (ETH_SLAVE). */
    esp_netif_t* netifSlave()  const { return _netif_slave; }

private:
    esp_err_t initOne(gpio_num_t cs, gpio_num_t intr, int clock_mhz,
                      const char* hostname,
                      esp_netif_t** out_netif,
                      esp_eth_handle_t* out_eth);

    esp_netif_t*     _netif_master = nullptr;
    esp_netif_t*     _netif_slave  = nullptr;
    esp_eth_handle_t _eth_master   = nullptr;
    esp_eth_handle_t _eth_slave    = nullptr;
    bool             _ok           = false;
};