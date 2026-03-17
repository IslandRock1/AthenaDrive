#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * DRV8323S SPI driver for ESP-IDF
 *
 * SPI frame (16-bit, MSB first):
 *   Bit 15    : R/W  (1 = read, 0 = write)
 *   Bits 14:11: address (4 bits)
 *   Bits 10:0 : data   (11 bits)
 *
 * SPI mode: CPOL=0, CPHA=1  (Mode 1)
 *   – SCLK idles low
 *   – Data captured on falling SCLK edge, propagated on rising SCLK edge
 *   – nSCS must be high for ≥ 400 ns between frames
 */

/** Pin configuration passed to drv8323_init(). */
typedef struct {
    spi_host_device_t spi_host;   /**< e.g. SPI2_HOST (HSPI) or SPI3_HOST (VSPI) */
    gpio_num_t        mosi;       /**< SDI  pin */
    gpio_num_t        miso;       /**< SDO  pin */
    gpio_num_t        sclk;       /**< SCLK pin */
    gpio_num_t        cs;         /**< nSCS pin (chip-select, active low) */
    gpio_num_t        enable;     /**< ENABLE pin (-1 if not connected) */
    int               spi_clock_hz; /**< SPI clock frequency in Hz (max ~10 MHz typical) */
} drv8323_config_t;

/** Opaque device handle. */
typedef struct drv8323_dev_t *drv8323_handle_t;

/**
 * @brief  Initialise the SPI bus and add the DRV8323S as a device.
 *
 * @param[in]  config   Pin and bus configuration.
 * @param[out] out_dev  Handle to use for subsequent calls.
 * @return ESP_OK on success.
 */
esp_err_t drv8323_init(const drv8323_config_t *config, drv8323_handle_t *out_dev);

/**
 * @brief  Free all resources associated with the handle.
 */
esp_err_t drv8323_deinit(drv8323_handle_t dev);

/**
 * @brief  Read a single register.
 *
 * @param[in]  dev    Device handle.
 * @param[in]  addr   Register address (0x00–0x07).
 * @param[out] data   11-bit register value.
 * @return ESP_OK on success.
 */
esp_err_t drv8323_read_reg(drv8323_handle_t dev, uint8_t addr, uint16_t *data);

/**
 * @brief  Write a single register.
 *
 * @param[in]  dev    Device handle.
 * @param[in]  addr   Register address (0x02–0x07, read-only registers are ignored by hardware).
 * @param[in]  data   11-bit value to write (bits 10:0 used).
 * @return ESP_OK on success.
 */
esp_err_t drv8323_write_reg(drv8323_handle_t dev, uint8_t addr, uint16_t data);

/**
 * @brief  Read-modify-write: apply a bitmask and new value.
 *
 * @param[in]  dev    Device handle.
 * @param[in]  addr   Register address.
 * @param[in]  mask   Bits to modify (1 = change, 0 = keep).
 * @param[in]  value  New bit values (only bits under mask are applied).
 * @return ESP_OK on success.
 */
esp_err_t drv8323_rmw_reg(drv8323_handle_t dev, uint8_t addr,
                           uint16_t mask, uint16_t value);

/**
 * @brief  Assert / de-assert the ENABLE pin.
 *         Has no effect if the ENABLE pin was configured as GPIO_NUM_NC.
 */
void drv8323_enable(drv8323_handle_t dev, bool enable);

/**
 * @brief  Clear all latched faults by momentarily setting CLR_FLT in Driver Control.
 */
esp_err_t drv8323_clear_faults(drv8323_handle_t dev);

/**
 * @brief  Read both status registers and print a human-readable fault summary.
 */
esp_err_t drv8323_print_faults(drv8323_handle_t dev);

void dump_all_registers(drv8323_handle_t dev);
esp_err_t configure_drv(drv8323_handle_t dev);

#ifdef __cplusplus
}
#endif