#include "w5500.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "w5500";

// SPI device handle — file-scoped, not exposed to users
static spi_device_handle_t s_spi = NULL;

// ---------------------------------------------------------------------------
// SPI low-level helpers
// ---------------------------------------------------------------------------

/*
 * W5500 SPI frame:
 *   Byte 0: Address high
 *   Byte 1: Address low
 *   Byte 2: Control  = [BSB(5 bits) | RWB(1 bit) | OM(2 bits)]
 *              BSB  = block select (common regs / socket regs / TX buf / RX buf)
 *              RWB  = 0 read, 1 write
 *              OM   = 0b00 fixed len 1, 0b10 fixed len 2, 0b11 variable len
 *   Byte 3+: Data
 *
 * We always use variable-length mode (OM = 0b11).
 */

#define CTRL_WRITE(bsb)  (((bsb) << 3) | 0x04 | 0x03)  // RWB=1, OM=11
#define CTRL_READ(bsb)   (((bsb) << 3) | 0x00 | 0x03)  // RWB=0, OM=11

void w5500_write_reg(uint16_t addr, uint8_t bsb, const uint8_t *data, uint16_t len)
{
    // Header is 3 bytes; allocate on stack for short transfers, heap for long
    uint8_t *tx = malloc(3 + len);
    if (!tx) {
        ESP_LOGE(TAG, "w5500_write_reg: malloc failed");
        return;
    }
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = CTRL_WRITE(bsb);
    memcpy(tx + 3, data, len);

    spi_transaction_t t = {
        .length    = (3 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(s_spi, &t);
    free(tx);
}

void w5500_read_reg(uint16_t addr, uint8_t bsb, uint8_t *data, uint16_t len)
{
    uint8_t *tx = calloc(1, 3 + len);  // TX needs to clock out zeros
    uint8_t *rx = malloc(3 + len);
    if (!tx || !rx) {
        ESP_LOGE(TAG, "w5500_read_reg: malloc failed");
        free(tx); free(rx);
        return;
    }
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = CTRL_READ(bsb);

    spi_transaction_t t = {
        .length    = (3 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_spi, &t);
    memcpy(data, rx + 3, len);
    free(tx); free(rx);
}

uint8_t w5500_read8(uint16_t addr, uint8_t bsb)
{
    uint8_t val;
    w5500_read_reg(addr, bsb, &val, 1);
    return val;
}

void w5500_write8(uint16_t addr, uint8_t bsb, uint8_t val)
{
    w5500_write_reg(addr, bsb, &val, 1);
}

uint16_t w5500_read16(uint16_t addr, uint8_t bsb)
{
    uint8_t buf[2];
    w5500_read_reg(addr, bsb, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

void w5500_write16(uint16_t addr, uint8_t bsb, uint16_t val)
{
    uint8_t buf[2] = { val >> 8, val & 0xFF };
    w5500_write_reg(addr, bsb, buf, 2);
}

// ---------------------------------------------------------------------------
// Public: init
// ---------------------------------------------------------------------------

esp_err_t w5500_init(void)
{
    // --- Hardware reset (if pin is wired) ---
    if (W5500_PIN_RST >= 0) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = (1ULL << W5500_PIN_RST),
            .mode         = GPIO_MODE_OUTPUT,
        };
        gpio_config(&rst_cfg);
        gpio_set_level(W5500_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(2));   // assert reset for 2 ms
        gpio_set_level(W5500_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(50));  // wait for W5500 PLL to stabilise
    }

    // --- Configure INT pin as input ---
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << W5500_PIN_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,  // INT is active-low, open-drain
    };
    gpio_config(&int_cfg);

    // --- Init SPI bus ---
    spi_bus_config_t bus = {
        .miso_io_num     = W5500_PIN_MISO,
        .mosi_io_num     = W5500_PIN_MOSI,
        .sclk_io_num     = W5500_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(W5500_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    // --- Add W5500 as SPI device ---
    // W5500 uses SPI mode 0 (CPOL=0, CPHA=0).
    // command_bits and address_bits are 0 because we encode everything
    // manually in the data buffer (addr high, addr low, control byte).
    spi_device_interface_config_t dev = {
        .clock_speed_hz = W5500_SPI_CLOCK_HZ,
        .mode           = 0,
        .spics_io_num   = W5500_PIN_CS,
        .queue_size     = 4,
        .command_bits   = 0,
        .address_bits   = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(W5500_SPI_HOST, &dev, &s_spi));

    // --- Soft reset via Mode register ---
    w5500_write8(W5500_REG_MR, W5500_BSB_COMMON_REG, 0x80);  // RST bit
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- Verify chip version ---
    uint8_t version = w5500_read8(W5500_REG_VERSIONR, W5500_BSB_COMMON_REG);
    if (version != 0x04) {
        ESP_LOGE(TAG, "W5500 version register mismatch: got 0x%02X (expected 0x04)", version);
        ESP_LOGE(TAG, "Check wiring — MISO/MOSI/SCLK/CS and power (3.3V)");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "W5500 detected, version=0x%02X", version);

    // --- Program MAC address ---
    uint8_t mac[] = W5500_MAC;
    w5500_write_reg(W5500_REG_SHAR, W5500_BSB_COMMON_REG, mac, 6);
    ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // --- Program IP address ---
    uint8_t ip[]  = W5500_IP;
    uint8_t gw[]  = W5500_GW;
    uint8_t sub[] = W5500_SUBNET;
    w5500_write_reg(W5500_REG_SIPR,  W5500_BSB_COMMON_REG, ip,  4);
    w5500_write_reg(W5500_REG_GAR,   W5500_BSB_COMMON_REG, gw,  4);
    w5500_write_reg(W5500_REG_SUBR,  W5500_BSB_COMMON_REG, sub, 4);
    ESP_LOGI(TAG, "IP:  %d.%d.%d.%d", ip[0],  ip[1],  ip[2],  ip[3]);
    ESP_LOGI(TAG, "GW:  %d.%d.%d.%d", gw[0],  gw[1],  gw[2],  gw[3]);
    ESP_LOGI(TAG, "SUB: %d.%d.%d.%d", sub[0], sub[1], sub[2], sub[3]);

    // --- Retry: 200 ms timeout, up to 3 retries ---
    w5500_write16(W5500_REG_RTR, W5500_BSB_COMMON_REG, 2000);  // unit = 100 µs
    w5500_write8 (W5500_REG_RCR, W5500_BSB_COMMON_REG, 3);

    // --- Unmask socket 0 interrupt globally ---
    w5500_write8(W5500_REG_SIMR, W5500_BSB_COMMON_REG, 0x01);  // socket 0 only

    ESP_LOGI(TAG, "W5500 init complete");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Public: link status
// ---------------------------------------------------------------------------

bool w5500_link_up(void)
{
    uint8_t phycfg = w5500_read8(W5500_REG_PHYCFGR, W5500_BSB_COMMON_REG);
    return (phycfg & W5500_PHYCFGR_LNK) != 0;
}

// ---------------------------------------------------------------------------
// ICMP helpers
// ---------------------------------------------------------------------------

static uint16_t icmp_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        sum += ((uint16_t)data[i] << 8) | data[i + 1];
    }
    if (len & 1) {
        sum += (uint16_t)data[len - 1] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return ~(uint16_t)sum;
}

// ICMP Echo Request packet (8-byte header + 32 bytes payload = 40 bytes)
// We keep it small so it fits well in the W5500 TX buffer.
#define PING_PAYLOAD_LEN 32
#define PING_PKT_LEN     (8 + PING_PAYLOAD_LEN)

static void build_icmp_echo_request(uint8_t *pkt, uint16_t seq)
{
    memset(pkt, 0, PING_PKT_LEN);
    pkt[0] = 8;    // Type: Echo Request
    pkt[1] = 0;    // Code: 0
    // pkt[2..3] = checksum (filled below)
    pkt[4] = 0;    // Identifier high
    pkt[5] = 1;    // Identifier low  (arbitrary, just something non-zero)
    pkt[6] = seq >> 8;
    pkt[7] = seq & 0xFF;
    // Fill payload with recognisable pattern
    for (int i = 0; i < PING_PAYLOAD_LEN; i++) {
        pkt[8 + i] = (uint8_t)(0x41 + (i % 26));  // 'A'..'Z' repeating
    }
    uint16_t csum = icmp_checksum(pkt, PING_PKT_LEN);
    pkt[2] = csum >> 8;
    pkt[3] = csum & 0xFF;
}

// ---------------------------------------------------------------------------
// Public: ping
// ---------------------------------------------------------------------------

esp_err_t w5500_ping(const uint8_t target_ip[4], uint32_t timeout_ms)
{
    const uint8_t sn = 0;  // Use socket 0
    uint8_t bsb_reg = W5500_BSB_Sn_REG(sn);
    uint8_t bsb_tx  = W5500_BSB_Sn_TX(sn);
    uint8_t bsb_rx  = W5500_BSB_Sn_RX(sn);
    static uint16_t seq = 0;

    // --- Close socket if it was left open ---
    w5500_write8(W5500_Sn_CR, bsb_reg, W5500_Sn_CR_CLOSE);
    vTaskDelay(pdMS_TO_TICKS(5));

    // --- Open socket in IPRAW mode, protocol 0x01 (ICMP) ---
    // In IPRAW mode, MR[3:0] = 0x03, and Sn_PROTO (0x0014) sets IP protocol number
    w5500_write8(W5500_Sn_MR, bsb_reg, W5500_Sn_MR_IPRAW);
    w5500_write8(0x0014, bsb_reg, 0x01);  // Sn_PROTO = 1 (ICMP)
    w5500_write8(W5500_Sn_CR, bsb_reg, W5500_Sn_CR_OPEN);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t status = w5500_read8(W5500_Sn_SR, bsb_reg);
    if (status != W5500_Sn_SR_IPRAW) {
        ESP_LOGE(TAG, "Socket 0 failed to open in IPRAW mode (SR=0x%02X)", status);
        return ESP_FAIL;
    }

    // --- Set destination IP ---
    w5500_write_reg(W5500_Sn_DIPR, bsb_reg, target_ip, 4);

    // --- Build ICMP Echo Request ---
    uint8_t pkt[PING_PKT_LEN];
    build_icmp_echo_request(pkt, ++seq);

    ESP_LOGI(TAG, "Pinging %d.%d.%d.%d (seq=%d) ...",
             target_ip[0], target_ip[1], target_ip[2], target_ip[3], seq);

    // --- Write packet to TX buffer ---
    // 1. Read current TX write pointer
    uint16_t tx_wr = w5500_read16(W5500_Sn_TX_WR, bsb_reg);

    // 2. Write packet data at tx_wr (the W5500 handles buffer wrap internally)
    w5500_write_reg(tx_wr, bsb_tx, pkt, PING_PKT_LEN);

    // 3. Advance TX write pointer
    w5500_write16(W5500_Sn_TX_WR, bsb_reg, tx_wr + PING_PKT_LEN);

    // 4. Issue SEND command
    w5500_write8(W5500_Sn_CR, bsb_reg, W5500_Sn_CR_SEND);

    // --- Wait for SEND_OK interrupt ---
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t ir = w5500_read8(W5500_Sn_IR, bsb_reg);
        if (ir & W5500_Sn_IR_SEND_OK) {
            w5500_write8(W5500_Sn_IR, bsb_reg, W5500_Sn_IR_SEND_OK);  // clear
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // --- Wait for RECV interrupt (Echo Reply) ---
    bool got_reply = false;
    while (esp_timer_get_time() < deadline) {
        // Also check INT pin — active low means data arrived
        if (gpio_get_level(W5500_PIN_INT) == 0) {
            uint8_t ir = w5500_read8(W5500_Sn_IR, bsb_reg);
            if (ir & W5500_Sn_IR_RECV) {
                got_reply = true;
                w5500_write8(W5500_Sn_IR, bsb_reg, W5500_Sn_IR_RECV);  // clear
                break;
            }
            // Clear global interrupt register so INT pin de-asserts
            uint8_t gir = w5500_read8(W5500_REG_IR, W5500_BSB_COMMON_REG);
            w5500_write8(W5500_REG_IR, W5500_BSB_COMMON_REG, gir);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (got_reply) {
        // Read and discard RX data (update RX read pointer so buffer doesn't fill)
        uint16_t rx_rsr = w5500_read16(W5500_Sn_RX_RSR, bsb_reg);
        uint16_t rx_rd  = w5500_read16(W5500_Sn_RX_RD,  bsb_reg);
        // In IPRAW mode the W5500 prepends a 4-byte info header (src IP) before
        // the ICMP packet, so the reply is rx_rsr bytes total.
        w5500_write16(W5500_Sn_RX_RD, bsb_reg, rx_rd + rx_rsr);
        w5500_write8(W5500_Sn_CR, bsb_reg, W5500_Sn_CR_RECV);  // release buffer

        int64_t rtt_us = deadline - esp_timer_get_time();
        int32_t rtt_ms = (int32_t)(timeout_ms) - (int32_t)(rtt_us / 1000);
        ESP_LOGI(TAG, "Reply from %d.%d.%d.%d: seq=%d time=%"PRId32" ms",
                 target_ip[0], target_ip[1], target_ip[2], target_ip[3],
                 seq, rtt_ms > 0 ? rtt_ms : 0);
    } else {
        ESP_LOGW(TAG, "Request timeout for seq=%d", seq);
    }

    // --- Close socket ---
    w5500_write8(W5500_Sn_CR, bsb_reg, W5500_Sn_CR_CLOSE);

    return got_reply ? ESP_OK : ESP_ERR_TIMEOUT;
}
