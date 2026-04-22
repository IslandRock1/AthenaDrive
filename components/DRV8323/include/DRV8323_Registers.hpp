#pragma once
#include <stdint.h>

// Register address
#define DRV_REG_FAULT_STATUS1   0x00  // R   – Fault Status 1
#define DRV_REG_VGS_STATUS2     0x01  // R   – VGS Status 2
#define DRV_REG_DRIVER_CTRL     0x02  // R/W – Driver Control
#define DRV_REG_GATE_DRIVE_HS   0x03  // R/W – Gate Drive HS
#define DRV_REG_GATE_DRIVE_LS   0x04  // R/W – Gate Drive LS
#define DRV_REG_OCP_CTRL        0x05  // R/W – OCP Control
#define DRV_REG_CSA_CTRL        0x06  // R/W – CSA Control
// 0x07 reserved

#define DRV_FS1_FAULT           (1 << 10)
#define DRV_FS1_VDS_OCP         (1 << 9)
#define DRV_FS1_GDF             (1 << 8)
#define DRV_FS1_UVLO            (1 << 7)
#define DRV_FS1_OTSD            (1 << 6)
#define DRV_FS1_VDS_HA          (1 << 5)
#define DRV_FS1_VDS_LA          (1 << 4)
#define DRV_FS1_VDS_HB          (1 << 3)
#define DRV_FS1_VDS_LB          (1 << 2)
#define DRV_FS1_VDS_HC          (1 << 1)
#define DRV_FS1_VDS_LC          (1 << 0)

#define DRV_FS2_SA_OC           (1 << 10)
#define DRV_FS2_SB_OC           (1 << 9)
#define DRV_FS2_SC_OC           (1 << 8)
#define DRV_FS2_OTW             (1 << 7)
#define DRV_FS2_CPUV            (1 << 6)
#define DRV_FS2_VGS_HA          (1 << 5)
#define DRV_FS2_VGS_LA          (1 << 4)
#define DRV_FS2_VGS_HB          (1 << 3)
#define DRV_FS2_VGS_LB          (1 << 2)
#define DRV_FS2_VGS_HC          (1 << 1)
#define DRV_FS2_VGS_LC          (1 << 0)

#define DRV_DC_DIS_CPUV         (1 << 9)
#define DRV_DC_DIS_GDF          (1 << 8)
#define DRV_DC_OTW_REP          (1 << 7)
#define DRV_DC_1PWM_COM         (1 << 6)
#define DRV_DC_1PWM_DIR         (1 << 5)
#define DRV_DC_COAST            (1 << 4)
#define DRV_DC_BRAKE            (1 << 3)
#define DRV_DC_CLR_FLT          (1 << 0)

#define DRV_PWM_MODE_SHIFT      5
#define DRV_PWM_MODE_MASK       (0x3 << DRV_PWM_MODE_SHIFT)
#define DRV_PWM_MODE_6X         (0x0 << DRV_PWM_MODE_SHIFT)
#define DRV_PWM_MODE_3X         (0x1 << DRV_PWM_MODE_SHIFT)
#define DRV_PWM_MODE_1X         (0x2 << DRV_PWM_MODE_SHIFT)
#define DRV_PWM_MODE_IND        (0x3 << DRV_PWM_MODE_SHIFT)

#define DRV_LOCK_SHIFT          8
#define DRV_LOCK_MASK           (0x7 << DRV_LOCK_SHIFT)
#define DRV_LOCK_UNLOCK         (0x3 << DRV_LOCK_SHIFT)
#define DRV_LOCK_LOCK           (0x6 << DRV_LOCK_SHIFT)

#define DRV_CSA_FET             (1 << 10)
#define DRV_CSA_VREF_DIV        (1 << 9)
#define DRV_CSA_LS_REF          (1 << 8)

#define DRV_CSA_GAIN_SHIFT      6
#define DRV_CSA_GAIN_MASK       (0x3 << DRV_CSA_GAIN_SHIFT)
#define DRV_CSA_GAIN_5          (0x0 << DRV_CSA_GAIN_SHIFT)
#define DRV_CSA_GAIN_10         (0x1 << DRV_CSA_GAIN_SHIFT)
#define DRV_CSA_GAIN_20         (0x2 << DRV_CSA_GAIN_SHIFT)
#define DRV_CSA_GAIN_40         (0x3 << DRV_CSA_GAIN_SHIFT)
#define DRV_CSA_DIS_SEN         (1 << 5)
#define DRV_CSA_CAL_A           (1 << 4)
#define DRV_CSA_CAL_B           (1 << 3)
#define DRV_CSA_CAL_C           (1 << 2)

#define DRV_CSA_SEN_LVL_SHIFT   0
#define DRV_CSA_SEN_LVL_MASK    (0x3 << DRV_CSA_SEN_LVL_SHIFT)

#define DRV_SPI_READ            (1 << 15)
#define DRV_SPI_WRITE           (0 << 15)
#define DRV_SPI_ADDR(a)         (((a) & 0xF) << 11)
#define DRV_SPI_DATA(d)         ((d) & 0x7FF)
#define DRV_SPI_DATA_MASK       0x7FF