
#pragma once
#include "driver/spi_master.h"
//#include "driver/gpio.h"
//#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"


// AMS AS5048A
class Encoder {
public:
    explicit Encoder(spi_device_handle_t spi_handle) 
        : _spi_handle(spi_handle), _prevRawAngle(0), _cumAngle(0.0f) {}
    
    esp_err_t begin();
    esp_err_t update();
    esp_err_t deinit();

    uint16_t getRawAngle();       // 0-16384 (14-bit)
    float getAngleRad() { return static_cast<float>(getRawAngle()) * (2 * 3.14159265359f / 16384.0f); }     // 0-2pi
    float getAngleDeg() { return static_cast<float>(getRawAngle()) * (360.0f / 16384.0f); }          // 0-360 degrees
    float getCumAngleRad() { return _cumAngle; }       // cumulative angle

    void setZero();
    esp_err_t clearErrors();

    struct Diagnostics {
        bool error_flag;                // something wrong
        bool parity_error;              // error in communication
        uint8_t agc;                    // 8-30 = good magnet
        bool magnet_ok() const { return agc >= 8 && agc <= 30; }
    };

    esp_err_t getDiagnostics(Diagnostics& diag);

private:
    esp_err_t readRegister(uint16_t addr, uint16_t* result);
    uint16_t applyParity(uint16_t word);
    uint16_t applyZeroOffset(uint16_t raw) const;
    float calcDeltaAngle(uint16_t curr, uint16_t prev) const;

    spi_device_handle_t _spi_handle;
    uint16_t _zeroOffset;
    mutable uint16_t _prevRawAngle;
    mutable float _cumAngle;

    static constexpr const char* TAG_ENCODER = "AS5048A";
};
