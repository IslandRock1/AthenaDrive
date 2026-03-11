#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Pinout.hpp"
#include "SpiManager.hpp"
#include "Encoder.hpp"

extern "C" void app_main(void)
{
    // SPI + Encoder
    SpiManager spi(SPI2_HOST, SPI_CLK_0, SPI_MOSI_0, SPI_MISO_0);
    spi.init();
    
    spi_device_handle_t enc_handle;
    spi.addDevice(CHIP_SELECT_ENCODER, 1000000, 1, 2, &enc_handle);
    
    Encoder encoder(enc_handle);
    encoder.begin();
    Encoder::Diagnostics diag;
    encoder.getDiagnostics(diag);
    ESP_LOGI("DIAG", "Error: %d | AGC: %d | Magnet OK: %d",
                diag.error_flag, diag.agc, diag.magnet_ok());
    
while (1) {
    encoder.update();  // Oppdater encoder
    uint16_t fresh = encoder.getRawAngle();  // Les ny vinkel
    ESP_LOGI("ENCODER", "Fresh=%d", fresh);
    vTaskDelay(pdMS_TO_TICKS(100));
}



}