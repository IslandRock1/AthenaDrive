#include <stdio.h>
#include "OneshotADC.hpp"

const static char *TAG = "OneshotADC";

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

OneshotADC::OneshotADC() {
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));

    do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_3, ADC_ATTEN_DB_11, &adc1_cali_chan0_handle);
    do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, ADC_ATTEN_DB_11, &adc1_cali_chan1_handle);
    do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_5, ADC_ATTEN_DB_11, &adc1_cali_chan2_handle);
}

int OneshotADC::getA() {
    return voltageA.load();
}

int OneshotADC::getB() {
    return voltageB.load();
}

int OneshotADC::getC() {
    return voltageC.load();
}

void OneshotADC::startTasks() {
    xTaskCreatePinnedToCore(taskAEntry, "adcA", 4096, this, 10, nullptr, 1);
    xTaskCreatePinnedToCore(taskBEntry, "adcB", 4096, this, 10, nullptr, 1);
    xTaskCreatePinnedToCore(taskCEntry, "adcC", 4096, this, 10, nullptr, 1);
}

void OneshotADC::taskAEntry(void *arg) {
    auto *self = static_cast<OneshotADC *>(arg);
    self->taskA();
}

void OneshotADC::taskBEntry(void *arg) {
    auto *self = static_cast<OneshotADC *>(arg);
    self->taskB();
}

void OneshotADC::taskCEntry(void *arg) {
    auto *self = static_cast<OneshotADC *>(arg);
    self->taskC();
}

void OneshotADC::taskA() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readA();
    }
}

void OneshotADC::taskB() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readB();
    }
}

void OneshotADC::taskC() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readC();
    }
}

void OneshotADC::readA() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &rawA));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, rawA, &calcA));
    voltageA.store(calcA);
}

void OneshotADC::readB() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &rawB));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, rawB, &calcB));
    voltageB.store(calcB);
}

void OneshotADC::readC() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &rawC));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan2_handle, rawC, &calcC));
    voltageC.store(calcC);
}