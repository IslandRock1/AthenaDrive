
#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "SpiManagerPrimary.hpp"
#include "OneshotADC.hpp"
#include "Controller.hpp"
#include "PID.hpp"
#include "MCPWM.hpp"
#include "LowpassFilter.hpp"
#include "SerialComm.hpp"
#include "GlobalVariableManager.hpp"

extern gptimer_handle_t timer;
extern TaskHandle_t control_task_handle;
extern OneshotADC oneshotADC;

struct MotorTaskConfig {
    gpio_num_t MOSI;
    gpio_num_t MISO;
    gpio_num_t CLK;

    gpio_num_t CHIP_SELECT_ENCODER;
    gpio_num_t CHIP_SELECT_MOTORDRIVER;
    gpio_num_t MOTOR_LOW_A;
    gpio_num_t MOTOR_LOW_B;
    gpio_num_t MOTOR_LOW_C;
    gpio_num_t MOTOR_HIGH_A;
    gpio_num_t MOTOR_HIGH_B;
    gpio_num_t MOTOR_HIGH_C;
};

void setupSPI(SpiManagerPrimary &spiManager, MotorTaskConfig config);
void beginEncoder(SpiManagerPrimary &spiManager, MotorTaskConfig config);
void beginMotorDriver(SpiManagerPrimary &spiManager, MotorTaskConfig config);
void pwm_stuff(Mcpwm &mcpwm, MotorTaskConfig config);

bool timerCallback(
    gptimer_handle_t timer,
    gptimer_alarm_event_data_t *edata,
    void *user_ctx);

void initTimer(TaskHandle_t task_handle);
void IRAM_ATTR realTimeTask(void *pvParameters);