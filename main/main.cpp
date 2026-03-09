#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "I2CManager.hpp"
#include "ContinuousADC.hpp"
#include "PID.hpp"

extern "C" void app_main(void)
{
    ContinuousADC continuousAdc;

    PID_Reg pid{1, 0, 0};
    float value = pid.update(1, 1);
    printf("PID value: %lf \n", value);

    uint32_t numReadings = 0;
    int64_t startTime = esp_timer_get_time();

    while (1) {
        
        int phaseA;
        int phaseB;
        int phaseC;

        continuousAdc.getReading(phaseA, phaseB, phaseC);
        numReadings++;

        int64_t timeNow = esp_timer_get_time();
        if (timeNow - startTime > 1000000) {
            printf("Num readings: %li | Time: %lli us. Phases: %i, %i, %i.\n", numReadings, (timeNow - startTime), phaseA, phaseB, phaseC);
            startTime = timeNow;
            numReadings = 0;
        }
        
    }
}