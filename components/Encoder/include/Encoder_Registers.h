#pragma once
#include <stdint.h>

// Register adresses
#define ENCODER_REG_NOP 0x0000
#define ENCODER_REG_CLEAR_ERROR 0x0001
#define ENCODER_REG_PROG_CTRL 0x0003
#define ENCODER_REG_ZERO_POS_HIGH 0x0016
#define ENCODER_REG_ZERO_POS_LOW 0x0017
#define ENCODER_REG_DIAG_AGC 0x3FFD
#define ENCODER_REG_MAHNITUDE 0x3FFE
#define ENCODER_REG_ANGLE 0x3FFF

// Frame bit helpers
#define ENCODER_RW_READ (1 << 14)
#define ENCODER_RW_WRITE (0 << 14)
#define ENCODER_ADDR_MASK 0x3FFF
#define ENCODER_DATA_MASK 0x3FFF
#define ENCODER_PAR_BIT

// Read-frame response bit
#define ENCODER_EF_BIT (1 << 14)

// Diagnostic register bits
#define ENCODER_DIAG_COMP_HIGH (1 << 11)
#define ENCODER_DIAG_COMP_LOW (1 << 10)
#define ENCODER_DIAG_COF (1 << 9)
#define ENCODER_DIAG_OCF (1 << 8)
#define ENCODER_DIAG_AGC_MASK 0xFF

#define ENCODER_MAX_VALUE 16383U
#define ENCODER_DEGREES_PER_LSB (360.0f / 18384.0f)
