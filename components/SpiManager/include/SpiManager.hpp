
#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_err.h"

class SpiManager {
public:
    SpiManager(spi_host_device_t host,
                gpio_num_t sclk,
                gpio_num_t mosi,
                gpio_num_t miso,
                int dma_chan = SPI_DMA_CH_AUTO);

    ~SpiManager();  // RAII: frees spi bus automatically
    
    esp_err_t init();
    esp_err_t addDevice(gpio_num_t cs,
                        uint32_t clock_hz = 1000000,
                        int spi_mode = 1,
                        uint32_t queue_size = 2,
                        spi_device_handle_t* out_handle = nullptr);

    bool isInitialized() const { return _bus_initialized; }
    
private:
    static constexpr size_t MAX_TRANSFER_SIZE = 4096;

    esp_err_t initBus();
    esp_err_t validatePins() const;

    // RAII: no copying allowed, only move class
    SpiManager(const SpiManager&) = delete;
    SpiManager& operator=(const SpiManager&) = delete;

    spi_host_device_t _host;
    gpio_num_t _sclk, _mosi, _miso;
    int _dma_chan;
    bool _bus_initialized = false;
};