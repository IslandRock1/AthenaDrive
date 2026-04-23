#include <stdio.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include <cmath>
#include "AS5048.hpp"
#include "AS5048_Registers.hpp"

static uint16_t applyParity(uint16_t word)
{
    uint16_t w = word & 0x7FFF;

    w ^= (w >> 8);
    w ^= (w >> 4);
    w ^= (w >> 2);
    w ^= (w >> 1);

    if (w & 1) {
        word |= AS5048_PAR_BIT;
    } else {
        word &= ~AS5048_PAR_BIT;
    }
    return word;
}

esp_err_t AS5048::begin(EncoderConfig config) {
    esp_err_t err = BaseSPI<EncoderConfig>::begin(config);

    uint16_t error = 0;
    readRegister(AS5048_REG_CLEAR_ERROR, error);

    return err;
}

esp_err_t AS5048::update(int32_t &rotations, float &angle, float &cumAngle, float &velocity) {
    uint16_t rawAngle = 0;
    esp_err_t err = readRegister(AS5048_REG_ANGLE, rawAngle);
    if (err != ESP_OK) return err;

    int16_t raw = static_cast<int16_t>(rawAngle & AS5048_DATA_MASK);    // 0-16383

    if (_firstUpdate) {
        _prevRaw = raw;
        _rotations = 0;
        _prevAngleRad = (raw / 16384.0f) * 2.0f * M_PI;
        _prevTime = esp_timer_get_time();
        _firstUpdate = false;

        rotations = 0;
        angle = _prevAngleRad;
        cumAngle = _prevAngleRad;
        velocity = 0.0f;
        return ESP_OK;
    }

    int16_t delta = raw - _prevRaw;
    if (delta > 8192) delta -= 16384;
    else if (delta < -8192) delta += 16384;

    _rotations += delta;
    _prevRaw = raw;

    // normalise angle [0, 2pi]
    float angleRad = (raw / 16384.0f) * 2.0f * M_PI;

    // cumulated angle
    float cumAngleRad = (_rotations / 16384.0f) * 2.0f * M_PI;

    // velocity
    int64_t now = esp_timer_get_time();
    float deltaT = (now - _prevTime) * 1e-6f;
    float deltaAng = cumAngleRad - _prevAngleRad;

    velocity = (deltaT > 0.0f) ? (deltaAng / deltaT) : 0.0f;
    _prevTime = now;
    _prevAngleRad = cumAngleRad;

    rotations = _rotations / 16384;
    angle = angleRad;
    cumAngle = cumAngleRad;
    return ESP_OK;
}

esp_err_t AS5048::readRegister(uint16_t address, uint16_t &data) {
    uint16_t cmd = AS5048_RW_READ | (address & AS5048_ADDR_MASK);
    cmd = applyParity(cmd);

    uint16_t response0 = 0;
    esp_err_t err0 = _spiTransfer16(cmd, response0);

    uint16_t nop = applyParity(0x0000);
    uint16_t response1 = 0;
    esp_err_t err = _spiTransfer16(nop, response1);

    data = response1 & AS5048_DATA_MASK;
    return err;
}

esp_err_t AS5048::writeRegister(uint16_t address, uint16_t data) {
    uint16_t cmd = AS5048_RW_WRITE | (address & AS5048_ADDR_MASK);
    cmd = applyParity(cmd);
    uint16_t response0 = 0;
    esp_err_t err = _spiTransfer16(cmd, response0);

    uint16_t data_frame = data & AS5048_DATA_MASK;
    data_frame = applyParity(data_frame);
    uint16_t response1 = 0;
    err = _spiTransfer16(data_frame, response1);
    return err;
}