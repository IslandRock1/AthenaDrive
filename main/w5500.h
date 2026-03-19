#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "Pinout.hpp"

// ---------------------------------------------------------------------------
// Pin configuration — adjust to your wiring
// ---------------------------------------------------------------------------
#define W5500_PIN_MISO      SPI_MISO_1
#define W5500_PIN_MOSI      SPI_MOSI_1
#define W5500_PIN_SCLK      SPI_CLK_1
#define W5500_PIN_CS        CHIP_SELECT_W5500_0
#define W5500_PIN_INT       ETHERNET_INT_0
#define W5500_PIN_RST       GPIO_NUM_NC

#define W5500_SPI_HOST      SPI2_HOST
#define W5500_SPI_CLOCK_HZ  (20 * 1000 * 1000)  // 20 MHz (max 80 MHz)

// ---------------------------------------------------------------------------
// Network configuration — adjust to your network
// ---------------------------------------------------------------------------
#define W5500_MAC    { 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 }
#define W5500_IP     { 192, 168, 1, 200 }
#define W5500_GW     { 192, 168, 1, 1 }
#define W5500_SUBNET { 255, 255, 255, 0 }

// ---------------------------------------------------------------------------
// W5500 Common Register addresses
// ---------------------------------------------------------------------------
#define W5500_REG_MR          0x0000  // Mode
#define W5500_REG_GAR         0x0001  // Gateway Address (4 bytes)
#define W5500_REG_SUBR        0x0005  // Subnet Mask (4 bytes)
#define W5500_REG_SHAR        0x0009  // MAC Address (6 bytes)
#define W5500_REG_SIPR        0x000F  // Source IP (4 bytes)
#define W5500_REG_INTLEVEL    0x0013  // Interrupt Low Level Timer
#define W5500_REG_IR          0x0015  // Interrupt
#define W5500_REG_IMR         0x0016  // Interrupt Mask
#define W5500_REG_SIR         0x0017  // Socket Interrupt
#define W5500_REG_SIMR        0x0018  // Socket Interrupt Mask
#define W5500_REG_RTR         0x0019  // Retry Time (2 bytes)
#define W5500_REG_RCR         0x001B  // Retry Count
#define W5500_REG_VERSIONR    0x0039  // Chip version (should read 0x04)
#define W5500_REG_PHYCFGR     0x002E  // PHY config / link status

// ---------------------------------------------------------------------------
// W5500 Socket Register offsets (relative to socket base)
// ---------------------------------------------------------------------------
#define W5500_Sn_MR           0x0000  // Socket Mode
#define W5500_Sn_CR           0x0001  // Socket Command
#define W5500_Sn_IR           0x0002  // Socket Interrupt
#define W5500_Sn_SR           0x0003  // Socket Status
#define W5500_Sn_PORT         0x0004  // Source Port (2 bytes)
#define W5500_Sn_DIPR         0x000C  // Destination IP (4 bytes)
#define W5500_Sn_DPORT        0x0010  // Destination Port (2 bytes)
#define W5500_Sn_TX_FSR       0x0020  // TX Free Size (2 bytes)
#define W5500_Sn_TX_RD        0x0022  // TX Read Pointer (2 bytes)
#define W5500_Sn_TX_WR        0x0024  // TX Write Pointer (2 bytes)
#define W5500_Sn_RX_RSR       0x0026  // RX Received Size (2 bytes)
#define W5500_Sn_RX_RD        0x0028  // RX Read Pointer (2 bytes)
#define W5500_Sn_RX_WR        0x002A  // RX Write Pointer (2 bytes)
#define W5500_Sn_IMR          0x002C  // Socket Interrupt Mask

// Socket Mode Register values
#define W5500_Sn_MR_CLOSED    0x00
#define W5500_Sn_MR_TCP       0x01
#define W5500_Sn_MR_UDP       0x02
#define W5500_Sn_MR_IPRAW     0x03   // Raw IP — needed for ICMP ping

// Socket Command Register values
#define W5500_Sn_CR_OPEN      0x01
#define W5500_Sn_CR_CLOSE     0x10
#define W5500_Sn_CR_SEND      0x20
#define W5500_Sn_CR_RECV      0x40

// Socket Status Register values
#define W5500_Sn_SR_CLOSED    0x00
#define W5500_Sn_SR_IPRAW     0x32

// Socket Interrupt flags
#define W5500_Sn_IR_SEND_OK   0x10
#define W5500_Sn_IR_RECV      0x04

// PHY link bit
#define W5500_PHYCFGR_LNK     (1 << 0)

// ---------------------------------------------------------------------------
// Block select codes (used in the SPI control byte)
// ---------------------------------------------------------------------------
#define W5500_BSB_COMMON_REG  0x00   // Common registers
#define W5500_BSB_Sn_REG(n)  ((n) * 4 + 1)   // Socket n registers
#define W5500_BSB_Sn_TX(n)   ((n) * 4 + 2)   // Socket n TX buffer
#define W5500_BSB_Sn_RX(n)   ((n) * 4 + 3)   // Socket n RX buffer

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Initialise SPI bus, reset W5500, configure MAC/IP/GW/Subnet,
 *        and verify chip version.
 * @return ESP_OK on success
 */
esp_err_t w5500_init(void);

/**
 * @brief Read the link status from the PHY config register.
 * @return true if Ethernet cable is connected and link is up
 */
bool w5500_link_up(void);

/**
 * @brief Send an ICMP Echo Request (ping) to the given IP address
 *        and wait for a reply.
 *
 * Uses socket 0 in IPRAW mode (protocol 0x01 = ICMP).
 *
 * @param target_ip  4-byte target IP (e.g. {192,168,1,1})
 * @param timeout_ms How long to wait for a reply (milliseconds)
 * @return ESP_OK if a reply was received, ESP_ERR_TIMEOUT otherwise
 */
esp_err_t w5500_ping(const uint8_t target_ip[4], uint32_t timeout_ms);

// ---------------------------------------------------------------------------
// Low-level SPI register access (also useful for debugging)
// ---------------------------------------------------------------------------
void     w5500_write_reg(uint16_t addr, uint8_t bsb, const uint8_t *data, uint16_t len);
void     w5500_read_reg (uint16_t addr, uint8_t bsb, uint8_t *data, uint16_t len);
uint8_t  w5500_read8    (uint16_t addr, uint8_t bsb);
void     w5500_write8   (uint16_t addr, uint8_t bsb, uint8_t val);
uint16_t w5500_read16   (uint16_t addr, uint8_t bsb);
void     w5500_write16  (uint16_t addr, uint8_t bsb, uint16_t val);
