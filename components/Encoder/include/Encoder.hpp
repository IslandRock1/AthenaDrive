
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

    uint16_t getRawAngle();       // 0-16384 (14-bit)
    float getAngleRad() { return static_cast<float>(getRawAngle()) * (FULL_CIRCLE / 16384.0f); }     // 0-2pi
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
    static constexpr uint16_t ANGLE_CMD = 0x3FFF;
    static constexpr uint16_t DIAG_CMD = 0x3FFD;
    static constexpr uint16_t CLR_ERR_CMD = 0x0001;
    static constexpr float PI = 3.14159265359f;
    static constexpr float FULL_CIRCLE = 2.0f * PI;
    static constexpr uint16_t RESOLUTION = 16384;
    
    esp_err_t readRegister(uint16_t cmd, uint16_t* result);   // ta vekk const
    uint8_t calcParity(uint16_t value) const;
    uint16_t applyZeroOffset(uint16_t raw) const;
    float calcDeltaAngle(uint16_t curr, uint16_t prev) const;

    spi_device_handle_t _spi_handle = nullptr;
    uint16_t _zeroOffset = 0;
    mutable uint16_t _prevRawAngle = 0;
    mutable float _cumAngle = 0.0f;

    static constexpr const char* TAG_ENCODER = "AS5048A";
};
