#pragma once
#include <stdint.h>

// Register addresses
#define AS5048_REG_NOP              0x0000  // R   – NOP / dummy
#define AS5048_REG_CLEAR_ERROR      0x0001  // R   – Clear error flag
#define AS5048_REG_PROG_CTRL        0x0003  // R/W – Programming control
#define AS5048_REG_ZERO_POS_HI      0x0016  // R/W – Zero position high byte
#define AS5048_REG_ZERO_POS_LO      0x0017  // R/W – Zero position low 6 bits
#define AS5048_REG_DIAG_AGC         0x3FFD  // R   – Diagnostics + AGC
#define AS5048_REG_MAGNITUDE        0x3FFE  // R   – CORDIC magnitude
#define AS5048_REG_ANGLE            0x3FFF  // R   – Angle (with zero offset)

#define AS5048_RW_READ              (1 << 14)
#define AS5048_RW_WRITE             (0 << 14)
#define AS5048_ADDR_MASK            0x3FFF
#define AS5048_DATA_MASK            0x3FFF
#define AS5048_PAR_BIT              (1 << 15)

#define AS5048_EF_BIT               (1 << 14)

#define AS5048_DIAG_COMP_HIGH       (1 << 11)
#define AS5048_DIAG_COMP_LOW        (1 << 10)
#define AS5048_DIAG_COF             (1 << 9)
#define AS5048_DIAG_OCF             (1 << 8)
#define AS5048_DIAG_AGC_MASK        0xFF

#define AS5048_MAX_VALUE            16383U
#define AS5048_DEGREES_PER_LSB      (360.0f / 16384.0f)