#include "drv8323.h"
#include "drv8323_regs.h"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "drv8323";

/* ── Internal device structure ─────────────────────────────────────────── */
struct drv8323_dev_t {
    spi_device_handle_t spi;
    spi_host_device_t   host;
    gpio_num_t          enable_pin;
};

/* ── Low-level SPI transfer ─────────────────────────────────────────────── */

/**
 * Perform one 16-bit SPI transaction.
 * tx_word  : word to clock out on MOSI (SDI)
 * rx_word  : word received on MISO (SDO), may be NULL
 */
static esp_err_t spi_transfer_16(drv8323_handle_t dev,
                                  uint16_t tx_word,
                                  uint16_t *rx_word)
{
    /* esp-idf SPI master uses byte arrays; send MSB first */
    uint8_t tx_buf[2] = { (tx_word >> 8) & 0xFF, tx_word & 0xFF };
    uint8_t rx_buf[2] = { 0, 0 };

    spi_transaction_t t = {
        .length    = 16,          /* bits */
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t err = spi_device_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(err));
        return err;
    }

    if (rx_word) {
        *rx_word = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
        /* Strip don't-care top 5 bits; keep only data[10:0] */
        *rx_word &= DRV_SPI_DATA_MASK;
    }
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t drv8323_init(const drv8323_config_t *config, drv8323_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(config && out_dev, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    /* Allocate handle */
    struct drv8323_dev_t *dev = calloc(1, sizeof(struct drv8323_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "malloc failed");
    dev->host       = config->spi_host;
    dev->enable_pin = config->enable;

    /* Initialise SPI bus */
    spi_bus_config_t buscfg = {
        .mosi_io_num   = config->mosi,
        .miso_io_num   = config->miso,
        .sclk_io_num   = config->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means bus already initialised – that's fine */
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    /* Add DRV8323S as SPI device
     * – CPOL=0, CPHA=1  → SPI mode 1
     * – CS is managed by the SPI driver (active low by default)
     * – positive_cs = 0 (nSCS is active low, which is the default)
     */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_hz > 0 ? config->spi_clock_hz : 1000000,
        .mode           = 1,          /* CPOL=0, CPHA=1 */
        .spics_io_num   = config->cs,
        .queue_size     = 1,
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
        /* nSCS needs ≥400 ns between words – cs_ena_posttrans adds hold time
         * at 1 MHz one cycle = 1 µs, so 1 cycle is more than enough.
         * Increase if running at higher clock speeds. */
        .cs_ena_posttrans = 1,
    };

    err = spi_bus_add_device(config->spi_host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    /* Configure ENABLE pin if provided */
    if (config->enable != GPIO_NUM_NC) {
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = (1ULL << config->enable),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_cfg);
        gpio_set_level(config->enable, 0);   /* start disabled */
    }

    *out_dev = dev;
    ESP_LOGI(TAG, "DRV8323S initialised on SPI host %d", config->spi_host);
    return ESP_OK;
}

esp_err_t drv8323_deinit(drv8323_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    spi_bus_remove_device(dev->spi);
    spi_bus_free(dev->host);
    free(dev);
    return ESP_OK;
}

esp_err_t drv8323_read_reg(drv8323_handle_t dev, uint8_t addr, uint16_t *data)
{
    ESP_RETURN_ON_FALSE(dev && data, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    uint16_t tx = DRV_SPI_READ | DRV_SPI_ADDR(addr) | 0x000;
    esp_err_t err = spi_transfer_16(dev, tx, data);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "READ  reg[0x%02X] = 0x%03X", addr, *data);
    }
    return err;
}

esp_err_t drv8323_write_reg(drv8323_handle_t dev, uint8_t addr, uint16_t data)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    uint16_t tx = DRV_SPI_WRITE | DRV_SPI_ADDR(addr) | DRV_SPI_DATA(data);
    uint16_t rx = 0;
    esp_err_t err = spi_transfer_16(dev, tx, &rx);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "WRITE reg[0x%02X] = 0x%03X (prev=0x%03X)", addr, data & DRV_SPI_DATA_MASK, rx);
    }
    return err;
}

esp_err_t drv8323_rmw_reg(drv8323_handle_t dev, uint8_t addr,
                            uint16_t mask, uint16_t value)
{
    uint16_t current = 0;
    esp_err_t err = drv8323_read_reg(dev, addr, &current);
    if (err != ESP_OK) return err;

    uint16_t updated = (current & ~mask) | (value & mask);
    return drv8323_write_reg(dev, addr, updated);
}

void drv8323_enable(drv8323_handle_t dev, bool enable)
{
    if (!dev || dev->enable_pin == GPIO_NUM_NC) return;
    gpio_set_level(dev->enable_pin, enable ? 1 : 0);
    ESP_LOGI(TAG, "ENABLE pin -> %s", enable ? "HIGH" : "LOW");
}

esp_err_t drv8323_clear_faults(drv8323_handle_t dev)
{
    /* Set CLR_FLT bit, then clear it */
    esp_err_t err = drv8323_rmw_reg(dev, DRV_REG_DRIVER_CTRL,
                                     DRV_DC_CLR_FLT, DRV_DC_CLR_FLT);
    if (err != ESP_OK) return err;
    /* CLR_FLT self-clears, but write it back explicitly for safety */
    return drv8323_rmw_reg(dev, DRV_REG_DRIVER_CTRL, DRV_DC_CLR_FLT, 0);
}

esp_err_t drv8323_print_faults(drv8323_handle_t dev)
{
    uint16_t fs1 = 0, fs2 = 0;
    esp_err_t err;

    err = drv8323_read_reg(dev, DRV_REG_FAULT_STATUS1, &fs1);
    if (err != ESP_OK) return err;
    err = drv8323_read_reg(dev, DRV_REG_VGS_STATUS2, &fs2);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "--- DRV8323S Fault Status ---");
    ESP_LOGI(TAG, "Fault Status 1 (0x00): 0x%03X", fs1);
    if (fs1 == 0) {
        ESP_LOGI(TAG, "  No faults");
    } else {
        if (fs1 & DRV_FS1_FAULT)   ESP_LOGW(TAG, "  [FAULT]   General fault");
        if (fs1 & DRV_FS1_VDS_OCP) ESP_LOGW(TAG, "  [VDS_OCP] VDS overcurrent");
        if (fs1 & DRV_FS1_GDF)     ESP_LOGW(TAG, "  [GDF]     Gate drive fault");
        if (fs1 & DRV_FS1_UVLO)    ESP_LOGW(TAG, "  [UVLO]    Undervoltage lockout");
        if (fs1 & DRV_FS1_OTSD)    ESP_LOGW(TAG, "  [OTSD]    Overtemperature shutdown");
        if (fs1 & DRV_FS1_VDS_HA)  ESP_LOGW(TAG, "  [VDS_HA]  VDS OCP phase A high-side");
        if (fs1 & DRV_FS1_VDS_LA)  ESP_LOGW(TAG, "  [VDS_LA]  VDS OCP phase A low-side");
        if (fs1 & DRV_FS1_VDS_HB)  ESP_LOGW(TAG, "  [VDS_HB]  VDS OCP phase B high-side");
        if (fs1 & DRV_FS1_VDS_LB)  ESP_LOGW(TAG, "  [VDS_LB]  VDS OCP phase B low-side");
        if (fs1 & DRV_FS1_VDS_HC)  ESP_LOGW(TAG, "  [VDS_HC]  VDS OCP phase C high-side");
        if (fs1 & DRV_FS1_VDS_LC)  ESP_LOGW(TAG, "  [VDS_LC]  VDS OCP phase C low-side");
    }

    ESP_LOGI(TAG, "VGS Status 2 (0x01):  0x%03X", fs2);
    if (fs2 == 0) {
        ESP_LOGI(TAG, "  No faults");
    } else {
        if (fs2 & DRV_FS2_SA_OC)   ESP_LOGW(TAG, "  [SA_OC]   SenseAmp A overcurrent");
        if (fs2 & DRV_FS2_SB_OC)   ESP_LOGW(TAG, "  [SB_OC]   SenseAmp B overcurrent");
        if (fs2 & DRV_FS2_SC_OC)   ESP_LOGW(TAG, "  [SC_OC]   SenseAmp C overcurrent");
        if (fs2 & DRV_FS2_OTW)     ESP_LOGW(TAG, "  [OTW]     Overtemperature warning");
        if (fs2 & DRV_FS2_CPUV)    ESP_LOGW(TAG, "  [CPUV]    Charge pump undervoltage");
        if (fs2 & DRV_FS2_VGS_HA)  ESP_LOGW(TAG, "  [VGS_HA]  VGS fault phase A high-side");
        if (fs2 & DRV_FS2_VGS_LA)  ESP_LOGW(TAG, "  [VGS_LA]  VGS fault phase A low-side");
        if (fs2 & DRV_FS2_VGS_HB)  ESP_LOGW(TAG, "  [VGS_HB]  VGS fault phase B high-side");
        if (fs2 & DRV_FS2_VGS_LB)  ESP_LOGW(TAG, "  [VGS_LB]  VGS fault phase B low-side");
        if (fs2 & DRV_FS2_VGS_HC)  ESP_LOGW(TAG, "  [VGS_HC]  VGS fault phase C high-side");
        if (fs2 & DRV_FS2_VGS_LC)  ESP_LOGW(TAG, "  [VGS_LC]  VGS fault phase C low-side");
    }
    ESP_LOGI(TAG, "-----------------------------");
    return ESP_OK;
}

void dump_all_registers(drv8323_handle_t dev)
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

esp_err_t configure_drv(drv8323_handle_t dev)
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

