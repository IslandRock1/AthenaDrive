#pragma once
#include <stdint.h>

// Register addresses
#define ENCODER_REG_NOP              0x0000  // R   – NOP / dummy
#define ENCODER_REG_CLEAR_ERROR      0x0001  // R   – Clear error flag
#define ENCODER_REG_PROG_CTRL        0x0003  // R/W – Programming control
#define ENCODER_REG_ZERO_POS_HI      0x0016  // R/W – Zero position high byte
#define ENCODER_REG_ZERO_POS_LO      0x0017  // R/W – Zero position low 6 bits
#define ENCODER_REG_DIAG_AGC         0x3FFD  // R   – Diagnostics + AGC
#define ENCODER_REG_MAGNITUDE        0x3FFE  // R   – CORDIC magnitude
#define ENCODER_REG_ANGLE            0x3FFF  // R   – Angle (with zero offset)

#define ENCODER_RW_READ              (1 << 14)
#define ENCODER_RW_WRITE             (0 << 14)
#define ENCODER_ADDR_MASK            0x3FFF
#define ENCODER_DATA_MASK            0x3FFF
#define ENCODER_PAR_BIT              (1 << 15)

#define ENCODER_EF_BIT               (1 << 14)

#define ENCODER_DIAG_COMP_HIGH       (1 << 11)
#define ENCODER_DIAG_COMP_LOW        (1 << 10)
#define ENCODER_DIAG_COF             (1 << 9)
#define ENCODER_DIAG_OCF             (1 << 8)
#define ENCODER_DIAG_AGC_MASK        0xFF

#define ENCODER_MAX_VALUE            16383U
#define ENCODER_DEGREES_PER_LSB      (360.0f / 16384.0f)