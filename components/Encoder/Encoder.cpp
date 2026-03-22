
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rom/ets_sys.h"
 
#include "Encoder.hpp"
#include "Encoder_Registers.h"


uint16_t Encoder::applyParity(uint16_t word) {
    uint16_t w = word & 0x7FFF;
    w ^= (w >> 8);
    w ^= (w >> 4);
    w ^= (w >> 2);
    w ^= (w >> 1);
    if (w & 1) {
        word |= (1 << 15);  // parity = 1
    } else {
        word &= ~(1 << 15); // parity = 0
    }
    return word;
}

esp_err_t Encoder::readRegister(uint16_t addr, uint16_t* result) {
    uint16_t cmd = ENCODER_RW_READ | (addr & ENCODER_ADDR_MASK);
    cmd = applyParity(cmd);

    uint8_t tx1[2] = { uint8_t(cmd >> 8), uint8_t(cmd & 0xFF) };
    uint8_t rx1[2] = { 0, 0 };
    spi_transaction_t t1 = {};
    t1.length = 16;
    t1.tx_buffer = tx1;
    t1.rx_buffer = rx1;

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(_spi_handle, &t1), TAG_ENCODER, "TX cmd failed");

    uint16_t nop = applyParity(ENCODER_REG_NOP);
    uint8_t tx2[2] = { uint8_t(nop >> 8), uint8_t(nop & 0xFF) };
    uint8_t rx2[2] = { 0, 0 };
    spi_transaction_t t2 = {};
    t2.length = 16;
    t2.tx_buffer = tx2;
    t2.rx_buffer = rx2;

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(_spi_handle, &t2), TAG_ENCODER, "RX data failed");

    uint16_t frame = (uint16_t(rx2[0]) << 8) | rx2[1];
    *result = frame & ENCODER_DATA_MASK;

    ESP_LOGD(TAG_ENCODER, "read reg[0x%04X] = 0x%04X%s", addr, *result, (frame & ENCODER_EF_BIT ? " [EF]" : ""));
    
    return ESP_OK;
}

esp_err_t Encoder::begin() {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(ENCODER_REG_ANGLE, &raw), TAG_ENCODER, "angle read failed");

    ESP_LOGI(TAG_ENCODER, "Encoder detected! Raw angle=0x%04X (%u)", raw, raw);
    _prevRawAngle = raw;
    
    return ESP_OK;
}

esp_err_t Encoder::deinit() {
    if (_spi_handle) {
        esp_err_t ret = spi_bus_remove_device(_spi_handle);
        _spi_handle = nullptr;
        ESP_LOGI(TAG_ENCODER, "Encoder deinitialised");
        return ret;
    }
    return ESP_OK;
}

esp_err_t Encoder::update() {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(ENCODER_REG_ANGLE, &raw), TAG_ENCODER, "update failed");

    uint16_t angle = applyZeroOffset(raw);
    _cumAngle += calcDeltaAngle(angle, _prevRawAngle);
    _prevRawAngle = angle;

    return ESP_OK;
}

uint16_t Encoder::getRawAngle() {
    return applyZeroOffset(_prevRawAngle);
}

uint16_t Encoder::applyZeroOffset(uint16_t raw) const {
    int32_t adjusted = static_cast<int32_t>(raw) - static_cast<int32_t>(_zeroOffset);
    return adjusted < 0 ? adjusted + 16384 : adjusted;
}

float Encoder::calcDeltaAngle(uint16_t curr, uint16_t prev) const {
    int32_t delta = static_cast<int32_t>(curr) - static_cast<int32_t>(prev);
    if (delta > 8192) delta -= 16384;
    else if (delta < -8192) delta += 16384;
    return static_cast<float>(delta) * (2 * 3.14159265359f / 16384.0f);
}

void Encoder::setZero() {
    _zeroOffset = getRawAngle();
    _cumAngle = 0.0f;
    ESP_LOGI(TAG_ENCODER, "Zero set to raw=%u", _zeroOffset);
}

esp_err_t Encoder::clearErrors() {
    uint16_t dummy;
    return readRegister(ENCODER_REG_CLEAR_ERROR, &dummy);
}

esp_err_t Encoder::getDiagnostics(Diagnostics& diag) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(readRegister(ENCODER_REG_DIAG_AGC, &raw), TAG_ENCODER, "diag failed");

    diag.error_flag = (raw & ENCODER_EF_BIT) != 0;
    diag.agc = raw & ENCODER_DIAG_AGC_MASK;
    return ESP_OK;
}
