#include <stdio.h>
#include "SpiManagerPrimary.hpp"

void SpiManagerPrimary::beginManager(SpiConfigPrimary config) {
    spi_bus_config_t busCfg = {
        .mosi_io_num   = config.MOSI,
        .miso_io_num   = config.MISO,
        .sclk_io_num   = config.CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    esp_err_t err = spi_bus_initialize(config.SPI_HOST, &busCfg, SPI_DMA_CH_AUTO);
}

void SpiManagerPrimary::beginEncoder(EncoderConfig config) {
    encoder.begin(config);
}

void SpiManagerPrimary::beginMotorDriver(MotorDriverConfig config) {
    motorDriver.begin(config);
    motorDriver.modifyBits(DRV_REG_CSA_CTRL, DRV_CSA_GAIN_MASK, DRV_CSA_GAIN_40);
    
}