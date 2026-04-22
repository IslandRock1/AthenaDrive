#include <stdio.h>
#include "SpiManagerPrimary.hpp"

void SpiManagerPrimary::begin(SpiConfigPrimary config) {
    spi_bus_config_t busCfg = {
        .mosi_io_num   = config.MOSI,
        .miso_io_num   = config.MISO,
        .sclk_io_num   = config.CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    SpiConfig configEnc = {
        .spiHost = SPI2_HOST,
        .cs = config.CS_Encoder,
        .spiClockHz = 1000 * 1000,
        .mode = 3,
    };
    encoder.begin(configEnc);

    SpiConfig configDRV = {
        .spiHost = SPI2_HOST,
        .cs = config.CS_MotorDriver,
        .spiClockHz = 100000,
        .mode = 1,
    };
    motorDriver.begin(configDRV);
}