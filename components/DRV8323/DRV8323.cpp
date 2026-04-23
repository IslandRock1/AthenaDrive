#include <stdio.h>
#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"

esp_err_t DRV8323::begin(MotorDriverConfig config) {
    LOW_A = config.LOW_A;
    LOW_B = config.LOW_B;
    LOW_C = config.LOW_C;

    gpio_set_direction(LOW_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(LOW_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(LOW_C, GPIO_MODE_OUTPUT);
    gpio_set_level(LOW_A, false);
    gpio_set_level(LOW_B, false);
    gpio_set_level(LOW_C, false);

    esp_err_t err = BaseSPI<MotorDriverConfig>::begin(config);

    modifyBits(DRV_REG_CSA_CTRL, DRV_CSA_GAIN_MASK, DRV_CSA_GAIN_40);
    modifyBits(DRV_REG_DRIVER_CTRL, 0b1100000, 0b0100000); // 3x PWM mode

    uint16_t mdrvData = 0;
    readRegister(0x02, mdrvData);
    if (mdrvData != 32) {
        printf("Motordriver in wrong mode.\n");
    }

    readRegister(0x06, mdrvData);
    // Should be 3x PWM mode.
    if (mdrvData != 707) {
        printf("Motordriver in wrong mode.\n");
    }

    return err;
}

void DRV8323::enable() {
    gpio_set_level(LOW_A, true);
    gpio_set_level(LOW_B, true);
    gpio_set_level(LOW_C, true);
}

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
