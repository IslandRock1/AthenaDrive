#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * AS5048A magnetic rotary encoder driver for ESP-IDF.
 *
 * The AS5048A shares the SPI bus with other devices (e.g. DRV8323S).
 * It uses the same SPI mode (CPOL=0, CPHA=1 = Mode 1), so both devices
 * can be on the same spi_host — only their CS pins differ.
 *
 * Protocol quirks to be aware of:
 *   1. PIPELINED: the chip returns the PREVIOUS command's result.
 *      Reading a register always takes two transactions.
 *   2. PARITY: bit 15 must be set so that bits [15:0] have even parity.
 *      Wrong parity sets the error flag inside the AS5048A.
 */

typedef struct as5048a_dev_t *as5048a_handle_t;

/**
 * @brief  Add AS5048A to an already-initialised SPI bus.
 *
 * Call spi_bus_initialize() once (e.g. inside drv8323_init) before this.
 * If the bus is shared with the DRV8323S, pass the same spi_host.
 *
 * @param[in]  spi_host     SPI host the bus was initialised on.
 * @param[in]  cs_pin       GPIO connected to the encoder's CSn (active low).
 * @param[in]  clock_hz     SPI clock in Hz (max ~10 MHz; 1 MHz safe for bring-up).
 * @param[out] out_dev      Handle for subsequent calls.
 */
esp_err_t as5048a_init(spi_host_device_t spi_host, gpio_num_t cs_pin,
                        int clock_hz, as5048a_handle_t *out_dev);

/** @brief Free resources. */
esp_err_t as5048a_deinit(as5048a_handle_t dev);

/**
 * @brief  Read a register (two SPI transactions due to pipeline).
 *
 * @param[in]  dev   Device handle.
 * @param[in]  addr  14-bit register address (use AS5048_REG_* constants).
 * @param[out] data  14-bit register value.
 * @return ESP_OK, or ESP_ERR_INVALID_RESPONSE if the error flag was set.
 */
esp_err_t as5048a_read_reg(as5048a_handle_t dev, uint16_t addr, uint16_t *data);

/**
 * @brief  Write a register (two SPI transactions; second clocks data in).
 */
esp_err_t as5048a_write_reg(as5048a_handle_t dev, uint16_t addr, uint16_t data);

/**
 * @brief  Read the 14-bit raw angle (0–16383 maps to 0–360°).
 */
esp_err_t as5048a_read_angle_raw(as5048a_handle_t dev, uint16_t *angle_raw);

/**
 * @brief  Read angle in degrees (0.0 – 359.978°).
 */
esp_err_t as5048a_read_angle_deg(as5048a_handle_t dev, float *angle_deg);

/**
 * @brief  Read diagnostics register and print a human-readable summary.
 *         Returns ESP_ERR_INVALID_STATE if COF (CORDIC overflow) is set.
 */
esp_err_t as5048a_print_diagnostics(as5048a_handle_t dev);

/**
 * @brief  Clear the error flag register inside the AS5048A.
 */
esp_err_t as5048a_clear_error(as5048a_handle_t dev);

#ifdef __cplusplus
}
#endif