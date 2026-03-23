#include "SpiManager.hpp"
#include "esp_log.h"

static const char* TAG_SPI = "SpiManager";

SpiManager::~SpiManager() {
    deinit();
}

SpiManager::SpiManager(SpiManager&& other) noexcept {
    spi_ = other.spi_;
    other.spi_ = nullptr;
}

SpiManager& SpiManager::operator=(SpiManager&& other) noexcept {
    if (this != &other) {
        deinit();
        spi_ = other.spi_;
        other.spi_ = nullptr;
    }
    return *this;
}

esp_err_t SpiManager::init(spi_host_device_t host,
                           gpio_num_t cs_pin,
                           int clock_hz)
{
    if (spi_) {
        deinit();
    }

    spi_device_interface_config_t devcfg = {
        .command_bits     = 0,
        .address_bits     = 0,
        .dummy_bits       = 0,
        .mode             = 3,  // CPOL=1, CPHA=1 – as in original code
        .clock_speed_hz   = clock_hz > 0 ? clock_hz : 1000000,
        .spics_io_num     = cs_pin,
        .queue_size       = 1,
        .pre_cb           = nullptr,
        .post_cb          = nullptr,
        .cs_ena_posttrans = 1,  // ≥350 ns CS high between frames
    };

    esp_err_t err = spi_bus_add_device(host, &devcfg, &spi_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SPI, "spi_bus_add_device: %s", esp_err_to_name(err));
        spi_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG_SPI, "SPI device initialised on host %d, CS GPIO%d",
             host, cs_pin);
    return ESP_OK;
}

void SpiManager::deinit() {
    if (spi_) {
        spi_bus_remove_device(spi_);
        spi_ = nullptr;
    }
}

esp_err_t SpiManager::transfer16(uint16_t tx, uint16_t* rx) {
    if (!spi_) return ESP_ERR_INVALID_STATE;

    uint8_t tx_buf[2] = { static_cast<uint8_t>((tx >> 8) & 0xFF),
                          static_cast<uint8_t>(tx & 0xFF) };
    uint8_t rx_buf[2] = { 0, 0 };

    spi_transaction_t t = {};
    t.length    = 16;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t err = spi_device_transmit(spi_, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SPI, "SPI transmit error: %s", esp_err_to_name(err));
        return err;
    }

    if (rx) {
        *rx = (static_cast<uint16_t>(rx_buf[0]) << 8) | rx_buf[1];
    }
    return ESP_OK;
}
