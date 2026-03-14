#include "as5048a.h"
#include "as5048a_regs.h"

#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>

static const char *TAG = "as5048a";

/* ── Internal structure ─────────────────────────────────────────────────── */
struct as5048a_dev_t {
    spi_device_handle_t spi;
};

/* ── Parity helper ───────────────────────────────────────────────────────── */
/**
 * Compute even parity over bits [14:0] of a 16-bit word.
 * Returns the full 16-bit word with bit 15 set correctly.
 */
static uint16_t apply_parity(uint16_t word)
{
    /* work on bits [14:0] only */
    uint16_t w = word & 0x7FFF;
    /* fold to get the XOR of all bits */
    w ^= (w >> 8);
    w ^= (w >> 4);
    w ^= (w >> 2);
    w ^= (w >> 1);
    /* if result LSB == 1 the number of 1-bits is odd → set par bit to make even */
    if (w & 1) {
        word |= AS5048_PAR_BIT;
    } else {
        word &= ~AS5048_PAR_BIT;
    }
    return word;
}

/* ── Low-level SPI ───────────────────────────────────────────────────────── */
static esp_err_t spi_xfer(struct as5048a_dev_t *dev,
                           uint16_t tx, uint16_t *rx)
{
    uint8_t tx_buf[2] = { (tx >> 8) & 0xFF, tx & 0xFF };
    uint8_t rx_buf[2] = { 0, 0 };

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t err = spi_device_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit error: %s", esp_err_to_name(err));
        return err;
    }

    if (rx) {
        *rx = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    }
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t as5048a_init(spi_host_device_t spi_host, gpio_num_t cs_pin,
                        int clock_hz, as5048a_handle_t *out_dev)
{
    if (!out_dev) return ESP_ERR_INVALID_ARG;

    struct as5048a_dev_t *dev = calloc(1, sizeof(struct as5048a_dev_t));
    if (!dev) return ESP_ERR_NO_MEM;

    /* Add encoder as a second device on the shared SPI bus.
     * Same mode as DRV8323S: CPOL=0, CPHA=1 (Mode 1).
     * CSn high-time ≥350 ns; at 1 MHz one cycle = 1 µs → 1 cycle is fine. */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz   = clock_hz > 0 ? clock_hz : 1000000,
        .mode             = 3,          /* CPOL=1, CPHA=1 – required by AS5048A */
        .spics_io_num     = cs_pin,
        .queue_size       = 1,
        .command_bits     = 0,
        .address_bits     = 0,
        .dummy_bits       = 0,
        .cs_ena_posttrans = 1,          /* ≥350 ns CSn high between frames */
    };

    esp_err_t err = spi_bus_add_device(spi_host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    *out_dev = dev;
    ESP_LOGI(TAG, "AS5048A initialised on SPI host %d, CS GPIO%d",
             spi_host, cs_pin);
    return ESP_OK;
}

esp_err_t as5048a_deinit(as5048a_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    spi_bus_remove_device(dev->spi);
    free(dev);
    return ESP_OK;
}

esp_err_t as5048a_read_reg(as5048a_handle_t dev, uint16_t addr, uint16_t *data)
{
    if (!dev || !data) return ESP_ERR_INVALID_ARG;

    /* Transaction 1: send READ command.
     * Frame: PAR | RWn=1 | addr[13:0] */
    uint16_t cmd = AS5048_RW_READ | (addr & AS5048_ADDR_MASK);
    cmd = apply_parity(cmd);

    uint16_t resp1 = 0;
    esp_err_t err = spi_xfer(dev, cmd, &resp1);
    if (err != ESP_OK) return err;

    /* Transaction 2: send NOP to clock out the actual data.
     * NOP frame is all zeros (parity of 0x0000 is even → PAR=0). */
    uint16_t nop = apply_parity(0x0000);
    uint16_t resp2 = 0;
    err = spi_xfer(dev, nop, &resp2);
    if (err != ESP_OK) return err;

    /* resp2 contains the answer to the READ command.
     * Bit 14 is the Error Flag. We ALWAYS extract the data first — it is
     * valid even when EF is set (EF just means a previous command had a
     * parity/framing error). Return the data then signal EF to the caller. */
    *data = resp2 & AS5048_DATA_MASK;
    ESP_LOGD(TAG, "READ  reg[0x%04X] = 0x%04X%s", addr, *data,
             (resp2 & AS5048_EF_BIT) ? " [EF]" : "");

    if (resp2 & AS5048_EF_BIT) {
        // ESP_LOGW(TAG, "Error flag set reading reg 0x%04X (resp=0x%04X)",
        //          addr, resp2);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t as5048a_write_reg(as5048a_handle_t dev, uint16_t addr, uint16_t data)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    /* Transaction 1: WRITE command with address.
     * Frame: PAR | RWn=0 | addr[13:0] */
    uint16_t cmd = AS5048_RW_WRITE | (addr & AS5048_ADDR_MASK);
    cmd = apply_parity(cmd);
    esp_err_t err = spi_xfer(dev, cmd, NULL);
    if (err != ESP_OK) return err;

    /* Transaction 2: data frame.
     * Frame: PAR | R=0 | data[13:0] */
    uint16_t data_frame = data & AS5048_DATA_MASK;
    data_frame = apply_parity(data_frame);
    uint16_t resp = 0;
    err = spi_xfer(dev, data_frame, &resp);
    if (err != ESP_OK) return err;

    if (resp & AS5048_EF_BIT) {
        ESP_LOGW(TAG, "Error flag set writing reg 0x%04X", addr);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGD(TAG, "WRITE reg[0x%04X] = 0x%04X", addr, data & AS5048_DATA_MASK);
    return ESP_OK;
}

esp_err_t as5048a_read_angle_raw(as5048a_handle_t dev, uint16_t *angle_raw)
{
    return as5048a_read_reg(dev, AS5048_REG_ANGLE, angle_raw);
}

esp_err_t as5048a_read_angle_deg(as5048a_handle_t dev, float *angle_deg)
{
    uint16_t raw = 0;
    esp_err_t err = as5048a_read_angle_raw(dev, &raw);
    /* Accept the data even if EF was set — data is valid, EF is a stale flag.
     * The caller should call as5048a_clear_error() once to clear it. */
    if (err != ESP_OK && err != ESP_ERR_INVALID_RESPONSE) return err;
    *angle_deg = (float)raw * AS5048_DEGREES_PER_LSB;
    /* Propagate EF so caller knows to clear it, but angle is populated */
    return err;
}

esp_err_t as5048a_print_diagnostics(as5048a_handle_t dev)
{
    uint16_t diag = 0;
    esp_err_t err = as5048a_read_reg(dev, AS5048_REG_DIAG_AGC, &diag);
    if (err != ESP_OK) return err;

    uint8_t agc = diag & AS5048_DIAG_AGC_MASK;

    ESP_LOGI(TAG, "--- AS5048A Diagnostics ---");
    ESP_LOGI(TAG, "  AGC value : %u  (%s)", agc,
             agc < 50  ? "strong field" :
             agc > 200 ? "weak field"   : "normal");
    ESP_LOGI(TAG, "  OCF       : %s", (diag & AS5048_DIAG_OCF)       ? "OK (offset comp done)" : "not ready");
    ESP_LOGI(TAG, "  COF       : %s", (diag & AS5048_DIAG_COF)       ? "OVERFLOW – angle invalid!" : "OK");
    ESP_LOGI(TAG, "  COMP_HIGH : %s", (diag & AS5048_DIAG_COMP_HIGH) ? "WARN: field too weak"  : "OK");
    ESP_LOGI(TAG, "  COMP_LOW  : %s", (diag & AS5048_DIAG_COMP_LOW)  ? "WARN: field too strong": "OK");
    ESP_LOGI(TAG, "---------------------------");

    return (diag & AS5048_DIAG_COF) ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t as5048a_clear_error(as5048a_handle_t dev)
{
    /* The AS5048A clears its error register when you READ register 0x0001.
     * We call read_reg twice:
     *   - First call sends the CLEAR_ERROR command; response is pipelined/stale.
     *   - Second call (NOP) clocks out the actual error register content.
     * read_reg already sets *data before checking EF, so this always works. */
    uint16_t error_content = 0;
    esp_err_t err = as5048a_read_reg(dev, AS5048_REG_CLEAR_ERROR, &error_content);
    if (err == ESP_ERR_INVALID_RESPONSE) {
        /* Expected — the EF bit was set, that's why we're clearing it.
         * The act of reading this register has already cleared the flag. */
        // ESP_LOGI(TAG, "Cleared encoder error register (content=0x%04X)", error_content);
        return ESP_OK;
    }
    return err;
}