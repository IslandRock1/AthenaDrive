
#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "SpiManager.hpp"


static const char* TAG_SPI = "SpiManager";

SpiManager::SpiManager(
    spi_host_device_t host,
    gpio_num_t sclk,
    gpio_num_t mosi,
    gpio_num_t miso,
    int dma_chan)

    : _host(host)
    , _sclk(sclk)
    , _mosi(mosi)
    , _miso(miso)
    , _dma_chan(dma_chan) {
        
        ESP_LOGI(TAG_SPI, "Init SPI%d (SCLK=%d, MOSI=%d, MISO=%d)", 
            _host, sclk, mosi, miso);
    }

SpiManager::~SpiManager() {
    if (_bus_initialized) {
        esp_err_t ret = spi_bus_free(_host);
        ESP_LOGW(TAG_SPI, "SPI%d bus auto-freed (RAII): %s", 
            _host, ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    }
}

esp_err_t SpiManager::validatePins() const {
    if (_sclk == GPIO_NUM_NC || _mosi == GPIO_NUM_NC || _miso == GPIO_NUM_NC) {
        ESP_LOGE(TAG_SPI, "Invalid pins: SCLK=%d MOSI=%d MISO=%d",
        _sclk, _mosi, _miso);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t SpiManager::initBus() {
    if (_bus_initialized) return ESP_OK;

    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = _miso;
    buscfg.mosi_io_num = _mosi;
    buscfg.sclk_io_num = _sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = MAX_TRANSFER_SIZE;

    esp_err_t ret = spi_bus_initialize(_host, &buscfg, _dma_chan);
    ESP_RETURN_ON_ERROR(ret, TAG_SPI, "SPI bus init failed: %s", esp_err_to_name(ret));
    
    _bus_initialized = true;
    ESP_LOGI(TAG_SPI, "SPI%d bus OK (max xfer: %d bytes)", _host, MAX_TRANSFER_SIZE);
    return ESP_OK;
}

esp_err_t SpiManager::init() {
    return initBus();
}

esp_err_t SpiManager::addDevice(
    gpio_num_t cs,
    uint32_t clock_hz,
    int spi_mode,
    uint32_t queue_size,
    spi_device_handle_t* out_handle) {

        ESP_RETURN_ON_FALSE(isInitialized(), ESP_ERR_INVALID_STATE, TAG_SPI, "Call init() first!");

        if (cs == GPIO_NUM_NC) {
            ESP_LOGE(TAG_SPI, "Invalid CS pin: %d", cs);
            return ESP_ERR_INVALID_ARG;
        }

        spi_device_interface_config_t devcfg = {};
        devcfg.mode = static_cast<uint8_t>(spi_mode);
        devcfg.clock_speed_hz = clock_hz;
        devcfg.spics_io_num = cs;
        devcfg.queue_size = queue_size;

        spi_device_handle_t handle;
        esp_err_t ret = spi_bus_add_device(_host, &devcfg, &handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SPI, "Device(CS=%d) failed: %s", cs, esp_err_to_name(ret));
            return ret;
        }
        
        if (out_handle) *out_handle = handle;
        ESP_LOGI(TAG_SPI, "Device OK: CS=%d @.1fMHz", cs, clock_hz/1e6f);
        return ESP_OK;
}