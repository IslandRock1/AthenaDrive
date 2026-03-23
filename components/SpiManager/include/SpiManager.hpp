#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"
#include "driver/gpio.h"

/**
 * Simple SPI device wrapper for 16‑bit transfers.
 */
class SpiManager {
public:
    SpiManager() = default;
    ~SpiManager();

    // Non-copyable
    SpiManager(const SpiManager&) = delete;
    SpiManager& operator=(const SpiManager&) = delete;

    // Movable
    SpiManager(SpiManager&& other) noexcept;
    SpiManager& operator=(SpiManager&& other) noexcept;

    esp_err_t init(spi_host_device_t host,
                   gpio_num_t cs_pin,
                   int clock_hz);

    void deinit();

    /**
     * Transmit a 16‑bit word and (optionally) receive the 16‑bit response.
     */
    esp_err_t transfer16(uint16_t tx, uint16_t* rx);

    spi_device_handle_t handle() const { return spi_; }

private:
    spi_device_handle_t spi_{nullptr};
};
