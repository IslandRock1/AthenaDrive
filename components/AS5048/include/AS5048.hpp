
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "BaseSPI.hpp"

struct EncoderConfig : public SpiConfig {}

class AS5048 : public BaseSPI {
public:

    esp_err_t update(int32_t &rotations, float &angle, float &cumAngle, float &velocity);
    esp_err_t readRegister(uint16_t address, uint16_t &data) override;
    esp_err_t writeRegister(uint16_t address, uint16_t data) override;
    esp_err_t modifyBits(uint16_t address, uint16_t mask, uint16_t value) override;

private:

    bool _firstUpdate = true;
    int16_t _prevRaw = 0;
    int32_t _rotations = 0;
    float _prevAngleRad = 0.0f;
    int64_t _prevTime = 0;
};