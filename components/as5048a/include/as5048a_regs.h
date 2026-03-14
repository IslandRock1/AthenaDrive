#pragma once
#include <stdint.h>

/**
 * AS5048A Register Definitions
 *
 * 16-bit SPI frame:
 *   Bit 15    : PAR  – even parity over bits [14:0]
 *   Bit 14    : RWn  – 1 = read, 0 = write
 *   Bits 13:0 : address (14-bit) or data (14-bit) depending on context
 *
 * IMPORTANT – pipelined protocol:
 *   The response to command N arrives during command N+1.
 *   To read a register: send READ cmd, then send NOP → data is in NOP response.
 *
 * SPI mode: CPOL=0, CPHA=1  (Mode 1) – same as DRV8323, safe to share bus.
 * CSn high-time between frames: ≥350 ns.
 */

/* ── Register Addresses ─────────────────────────────────────────────────── */
#define AS5048_REG_NOP              0x0000  /* R   – NOP / dummy              */
#define AS5048_REG_CLEAR_ERROR      0x0001  /* R   – Clear error flag          */
#define AS5048_REG_PROG_CTRL        0x0003  /* R/W – Programming control       */
#define AS5048_REG_ZERO_POS_HI      0x0016  /* R/W – Zero position high byte   */
#define AS5048_REG_ZERO_POS_LO      0x0017  /* R/W – Zero position low 6 bits  */
#define AS5048_REG_DIAG_AGC         0x3FFD  /* R   – Diagnostics + AGC         */
#define AS5048_REG_MAGNITUDE        0x3FFE  /* R   – CORDIC magnitude          */
#define AS5048_REG_ANGLE            0x3FFF  /* R   – Angle (with zero offset)  */

/* ── Frame bit helpers ───────────────────────────────────────────────────── */
#define AS5048_RW_READ              (1 << 14)
#define AS5048_RW_WRITE             (0 << 14)
#define AS5048_ADDR_MASK            0x3FFF   /* bits [13:0] */
#define AS5048_DATA_MASK            0x3FFF   /* bits [13:0] in response        */
#define AS5048_PAR_BIT              (1 << 15)

/* ── Read-frame response bits ────────────────────────────────────────────── */
#define AS5048_EF_BIT               (1 << 14)  /* Error flag in response       */

/* ── Diagnostics register (0x3FFD) bits ─────────────────────────────────── */
#define AS5048_DIAG_COMP_HIGH       (1 << 11)  /* Weak magnetic field          */
#define AS5048_DIAG_COMP_LOW        (1 << 10)  /* Strong magnetic field        */
#define AS5048_DIAG_COF             (1 << 9)   /* CORDIC overflow – angle invalid */
#define AS5048_DIAG_OCF             (1 << 8)   /* Offset compensation finished */
#define AS5048_DIAG_AGC_MASK        0xFF       /* AGC value bits [7:0]         */

/* ── Angle resolution ────────────────────────────────────────────────────── */
#define AS5048_MAX_VALUE            16383U     /* 2^14 - 1 */
#define AS5048_DEGREES_PER_LSB      (360.0f / 16384.0f)  /* 0.02197°/LSB      */