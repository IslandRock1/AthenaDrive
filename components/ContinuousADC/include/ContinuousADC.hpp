
#include "driver/gpio.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ADC_NUM_CHANNELS 3
#define EXAMPLE_ADC_FRAME_SIZE 12 // 12 bytes is 3 readings.
#define EXAMPLE_ADC_SAMPLE_FREQ_HZ 80000

/*

TODO: make this a background task.
That way, the adc can continually read,
storing the latest data in a variable.
Then control loop can get that data whenever.

*/

class ContinuousADC {
public:
    ContinuousADC();
    void getReading(int &phaseA_mv, int &phaseB_mv, int &phaseC_mv);

private:
    adc_continuous_handle_t _adc_handle = NULL;
    adc_cali_handle_t _adc_cali_handle = NULL;

    adc_channel_t _channel[EXAMPLE_ADC_NUM_CHANNELS] = {
        ADC_CHANNEL_3,  // Corresponds to GPIO4 on ADC1 for ESP32-S3
        ADC_CHANNEL_4,  // Corresponds to GPIO5 on ADC1 for ESP32-S3
        ADC_CHANNEL_5,  // Corresponds to GPIO6 on ADC1 for ESP32-S3
    };

    uint8_t _result_buffer[EXAMPLE_ADC_FRAME_SIZE] = {0};
    uint32_t _bytes_read = 0;
};