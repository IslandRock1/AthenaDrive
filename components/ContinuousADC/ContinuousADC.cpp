#include <stdio.h>
#include "esp_log.h"
#include "ContinuousADC.hpp"

ContinuousADC::ContinuousADC() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,      // Size of the internal DMA buffer
        .conv_frame_size = EXAMPLE_ADC_FRAME_SIZE, // Size of each conversion frame
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &_adc_handle));

    // ---- Step B: Configure ADC channels and sampling parameters ----
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = EXAMPLE_ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // Use only ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // Standard output format
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = EXAMPLE_ADC_NUM_CHANNELS;

    for (int i = 0; i < EXAMPLE_ADC_NUM_CHANNELS; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;    // Set attenuation for 0-3.3V range
        adc_pattern[i].channel = _channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // Use max resolution (12 or 13 bits)
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(_adc_handle, &dig_cfg));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &_adc_cali_handle));
    ESP_ERROR_CHECK(adc_continuous_start(_adc_handle));
}

void ContinuousADC::getReading(int &phaseA_mv, int &phaseB_mv, int &phaseC_mv) {
    esp_err_t ret = adc_continuous_read(_adc_handle, _result_buffer, EXAMPLE_ADC_FRAME_SIZE, &_bytes_read, 10);

    if (ret == ESP_OK) {
        for (int i = 0; i < _bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *data = (adc_digi_output_data_t*)&_result_buffer[i];
            int voltage_mv = 0;
            uint32_t raw_data;
            uint8_t channel_id;

            raw_data = data->type2.data;
            channel_id = data->type2.channel;

            if (adc_cali_raw_to_voltage(_adc_cali_handle, raw_data, &voltage_mv) == ESP_OK) {
                switch (channel_id) {
                    case 3:  // Assuming channel 3 is PHASE_C (GPIO4)
                        phaseC_mv = voltage_mv;
                        break;
                    case 4:  // Assuming channel 4 is PHASE_B (GPIO5)
                        phaseB_mv = voltage_mv;
                        break;
                    case 5:  // Assuming channel 5 is PHASE_A (GPIO6)
                        phaseA_mv = voltage_mv;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}