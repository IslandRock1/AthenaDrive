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

AS5048::AS5048(encoderConfig &config)
    : _spiHost(config.spiHost), _spiDevice(nullptr) {
    
    spi_device_interface_config_t devConfig = {
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
        .mode           = 3,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = config.spiClockHz > 0 ? config.spiClockHz : 1000000,
        .spics_io_num   = config.cs,
        .queue_size     = 1,
    };

    esp_err_t err = spi_bus_add_device(config.spiHost, &devConfig, &_spiDevice);
    if (err != ESP_OK) {
        _spiDevice = nullptr;
        ESP_LOGE("Encoder", "spi_bus_add_device failed: %s", esp_err_to_name(err));
    }
}

AS5048::~AS5048() {
    if (_spiDevice) {
        spi_bus_remove_device(_spiDevice);
    }
}

esp_err_t AS5048::update(int32_t &rotations, float &angle, float &cumAngle, float &velocity) {
    uint16_t rawAngle = 0;
    esp_err_t err = readRegister(AS5048_REG_ANGLE, &rawAngle);
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

esp_err_t AS5048::readRegister(uint16_t address, uint16_t *data) {
    uint16_t cmd = AS5048_RW_READ | (address & AS5048_ADDR_MASK);
    cmd = applyParity(cmd);

    uint16_t response0 = 0;
    esp_err_t err0 = spiTransfer16(cmd, &response0);

    uint16_t nop = applyParity(0x0000);
    uint16_t response1 = 0;
    esp_err_t err = spiTransfer16(nop, &response1);

    *data = response1 & AS5048_DATA_MASK;
    return err;
}

esp_err_t AS5048::writeRegister(uint16_t address, uint16_t data) {
    uint16_t cmd = AS5048_RW_WRITE | (address & AS5048_ADDR_MASK);
    cmd = applyParity(cmd);
    uint16_t response0 = 0;
    esp_err_t err = spiTransfer16(cmd, &response0);

    uint16_t data_frame = data & AS5048_DATA_MASK;
    data_frame = applyParity(data_frame);
    uint16_t response1 = 0;
    err = spiTransfer16(data_frame, &response1);
    return err;
}

esp_err_t AS5048::modifyBits(uint16_t address, uint16_t mask, uint16_t value) {
    uint16_t current = 0;
    esp_err_t err = readRegister(address, &current);
    if (err != ESP_OK) { return err; }

    uint16_t newValue = (current & ~mask) | (value & mask);
    err = writeRegister(address, newValue);
    return err;
}

esp_err_t AS5048::spiTransfer16(uint16_t tx, uint16_t *rx) {
    uint8_t txBuffer[2] = { static_cast<uint8_t>((tx >> 8) & 0xFF), static_cast<uint8_t>(tx & 0xFF) };
    uint8_t rxBuffer[2] = { 0, 0 };

    spi_transaction_t spiTransaction = {
        .length = 16,
        .tx_buffer = txBuffer,
        .rx_buffer = rxBuffer,
    };

    esp_err_t err = spi_device_transmit(_spiDevice, &spiTransaction);

    if (rx) {
        *rx = ((uint16_t)rxBuffer[0] << 8) | rxBuffer[1];
    }

    return err;
}