
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rom/ets_sys.h"
 
#include "Encoder.hpp"
#include "SpiManager.hpp"


esp_err_t Encoder::begin() {
    uint16_t result;
    ESP_RETURN_ON_ERROR(readRegister(ANGLE_CMD, &result), TAG_ENCODER, "angle reg test failed");
    ESP_RETURN_ON_FALSE((result & 0x4000) == 0, ESP_ERR_NOT_FOUND, TAG_ENCODER, "encoder not responding");
    ESP_LOGI(TAG_ENCODER, "Encoder AS5048A detected, AGC=%d", (result >> 10) & 0x1F);
    return ESP_OK;
}

esp_err_t Encoder::update() {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(ANGLE_CMD, &raw), TAG_ENCODER, "angle read failed");

    uint16_t angle = applyZeroOffset(raw & 0x3FFF);
    _cumAngle += calcDeltaAngle(angle, _prevRawAngle);
    _prevRawAngle = angle;

    return ESP_OK;
}

esp_err_t Encoder::readRegister(uint16_t cmd, uint16_t* result) {
    uint16_t tx_frame = (calcParity(cmd) << 15) | cmd;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = &tx_frame;

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(_spi_handle, &t), TAG_ENCODER, "TX failed");

    ets_delay_us(2);    // 2 us delay (datasheet requirement)

    uint16_t dummy = 0;
    t.tx_buffer = &dummy;
    t.rx_buffer = result;
    return spi_device_polling_transmit(_spi_handle, &t);
}

uint8_t Encoder::calcParity(uint16_t value) const {
    uint8_t parity = 0;
    for (int i = 0; i < 15; i++) parity ^= (value >> i) & 1;
    return parity;
}

uint16_t Encoder::applyZeroOffset(uint16_t raw) const {
    int32_t adjusted = static_cast<int32_t>(raw) - static_cast<int32_t>(_zeroOffset);
    return adjusted < 0 ? adjusted + RESOLUTION : adjusted;
}

float Encoder::calcDeltaAngle(uint16_t curr, uint16_t prev) const {
    int32_t delta = static_cast<int32_t>(curr) - static_cast<int32_t>(prev);
    if (delta > RESOLUTION/2) delta -= RESOLUTION;
    else if (delta < -RESOLUTION/2) delta += RESOLUTION;
    return static_cast<float>(delta) * (FULL_CIRCLE / RESOLUTION);
}

void Encoder::setZero() {
    _zeroOffset = getRawAngle();
    _cumAngle = 0.0f;
    ESP_LOGI(TAG_ENCODER, "Zero set to raw=%d", _zeroOffset);
}

esp_err_t Encoder::clearErrors() {
    uint16_t dummy;
    return readRegister(CLR_ERR_CMD, &dummy);
}

esp_err_t Encoder::getDiagnostics(Diagnostics& diag) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(DIAG_CMD, &raw), TAG_ENCODER, "diag read failed");

    diag.error_flag = raw & 0x4000;
    diag.parity_error = raw & 0x8000;
    diag.agc = (raw >> 10) & 0x1F;
    return ESP_OK;
}

uint16_t Encoder::getRawAngle() {
    return applyZeroOffset(_prevRawAngle);
}

// Fake roterende vinkel for DEBUG
//uint16_t Encoder::getRawAngle() {
  //  static uint32_t counter = 0;
  //  return (counter++ * 100) & 0x3FFF;  // Simulerer rotasjon
//}
