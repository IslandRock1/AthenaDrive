#include <stdio.h>
#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"

esp_err_t DRV8323::readRegister(uint16_t address, uint16_t &data) {
    uint16_t tx = DRV_SPI_READ | DRV_SPI_ADDR(address) | 0x0000;
    esp_err_t err = _spiTransfer16(tx, data);
    return err;
}

esp_err_t DRV8323::writeRegister(uint16_t address, uint16_t data) {
    uint16_t tx = DRV_SPI_WRITE | DRV_SPI_ADDR(address) | DRV_SPI_DATA(data);
    uint16_t rx = 0;
    esp_err_t err = _spiTransfer16(tx, rx);
    return err;
}
