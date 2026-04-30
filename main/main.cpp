#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdatomic.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "OneshotADC.hpp"
#include "Controller.hpp"
#include "PID.hpp"
#include "SerialComm.hpp"

#include "SpiManagerPrimary.hpp"
#include "MCPWM.hpp"

constexpr float PI_CONST = 3.1415926535897f;
constexpr float TWO_PI = 2.0f * PI_CONST;
constexpr float PI_DIV_2 = PI_CONST/ 2.0f;
constexpr float PI_7_DIV_6 = 7.0f * PI_CONST / 6.0f;
constexpr float PI_DIV_6 = PI_CONST / 6.0f;

gptimer_handle_t timer = NULL;
TaskHandle_t control_task_handle = NULL;

SpiManagerPrimary spiManager;
OneshotADC oneshotADC;

void setupSPI() {
    SpiConfigPrimary configPrimary = {
        .MOSI = SPI_MOSI_0,
        .MISO = SPI_MISO_0,
        .CLK = SPI_CLK_0,
        .SPI_HOST = SPI2_HOST,
    };
    spiManager.beginManager(configPrimary);
}

void beginEncoder() {
    EncoderConfig configEnc(SPI2_HOST, CHIP_SELECT_ENCODER, 10000000, 3);
    spiManager.beginEncoder(configEnc);
}

void beginMotorDriver() {
    MotorDriverConfig configDrv(SPI2_HOST, CHIP_SELECT_MOTOR_DRIVER, 100000, 1, MOTOR_LOW_A, MOTOR_LOW_B, MOTOR_LOW_C);
    spiManager.beginMotorDriver(configDrv);
}

Mcpwm pwm_stuff() {
    Mcpwm::Config pwmCfg = {};
    pwmCfg.pwm_a_gpio = MOTOR_HIGH_A;
    pwmCfg.pwm_b_gpio = MOTOR_HIGH_B;
    pwmCfg.pwm_c_gpio = MOTOR_HIGH_C;
    pwmCfg.n_sleep_gpio = GPIO_NUM_NC;

    Mcpwm mcpwm;
    mcpwm.init(pwmCfg);
    oneshotADC.startTask();
    mcpwm.register_adc_trigger(oneshotADC.taskHandle);
    mcpwm.enable();
    mcpwm.set_phase_voltages(-1.0, -1.0, -1.0);

    return mcpwm;
}
Mcpwm mcpwm;

static inline void atomic_store_float(atomic_uint *a, float f) {
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    atomic_store(a, bits);
}

static inline float atomic_load_float(atomic_uint *a) {
    uint32_t bits = atomic_load(a);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

atomic_bool secondaryTaskStarted = false;

atomic_int32_t rotationsGlob = 0;
atomic_uint angleGlob = 0.0;
atomic_uint cumAngleGlob = 0.0;
atomic_uint velocityGlob = 0.0;

atomic_uint avgVelocityGlob = 0;
atomic_uint avgStrengthGlob = 0;
atomic_uint avgLoopTimeGlob = 0;

float angleOffset = 0;
atomic_uint voltageGlob = 0;
atomic_uint drivingModeGlob = 0;

PI_Reg torquePI{0.0, 0.0};
atomic_uint torqueSetpointGlob = 0.0;
atomic_uint updateFreqTorqueGlob = 1;

PID_Reg velocityPID{0.0, 0.0, 0.0};
atomic_uint velocitySetpointGlob = 0.0;
atomic_uint updateFreqVelocityGlob = 10;

PID_Reg positionPID{0.0, 0.0, 0.0};
atomic_uint positionSetpointGlob = 0.0;
atomic_uint updateFreqPositionGlob = 100;

static bool IRAM_ATTR timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;

    TaskHandle_t task = (TaskHandle_t) user_ctx;

    vTaskNotifyGiveFromISR(task, &high_task_woken);

    return (high_task_woken == pdTRUE);
}

void init_timer(TaskHandle_t task_handle)
{
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz = 1 tick per µs
    };

    gptimer_new_timer(&config, &timer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };

    gptimer_register_event_callbacks(timer, &cbs, task_handle);

    gptimer_enable(timer);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 300,   // 150 µs
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_set_alarm_action(timer, &alarm_config);

    gptimer_start(timer);
}

void IRAM_ATTR realTimeTask(void *pvParameters) {
    setupSPI();
    beginEncoder();

    int32_t rotations;
    float angle, cumAngle, velocity;
    while (voltageGlob < 9000) {
        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        atomic_store_float(&angleGlob, angle);
        atomic_store_float(&cumAngleGlob, cumAngle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Wait 1 second.
    for (int i = 0; i < 100; i++) {
        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        atomic_store_float(&angleGlob, angle);
        atomic_store_float(&cumAngleGlob, cumAngle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    beginMotorDriver();

    mcpwm = pwm_stuff();
    mcpwm.set_phase_voltages(0.0, 0.0, 0.0);
    spiManager.motorDriver.enable();

    mcpwm.set_phase_voltages(0.04, -0.02, -0.02);
    // printf("Sat phase voltages.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    float numReadings = 0.0f;
    float sinSum = 0.0f;
    float cosSum = 0.0f;

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        numReadings += 1.0;

        sinSum += sin(angle);
        cosSum += cos(angle);
    }
    angleOffset = atan2(sinSum, cosSum);
    // printf("Finished reading offset.\n");

    static float sumVelocity = 0;
    static float sumStrength = 0;
    static float sumLoopTime = 0;
    static float numLoops = 0;

    float positionSetpoint = atomic_load_float(&positionSetpointGlob);
    float velocitySetpoint = atomic_load_float(&velocitySetpointGlob);
    float torqueSetpoint = atomic_load_float(&torqueSetpointGlob);

    float strengthOut = 0.0f;
    float strengthFilterAlpha = 0.001f;

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, true);
    bool ledState = true;

    secondaryTaskStarted.store(true);
    uint64_t iteration = 0;
    init_timer(control_task_handle);
    int64_t startTime = esp_timer_get_time();
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        iteration++;

        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        rotationsGlob = rotations;

        sumVelocity += velocity;
        angle -= angleOffset;
        float elPos = angle * 20.0f;
        while (elPos >= TWO_PI) { elPos -= TWO_PI; }
        while (elPos < 0) { elPos += TWO_PI; }
        Output output{};

        float deltaOffset = 0.0f;
        float strength = 0.0f;
        if ((drivingModeGlob > 2) && (iteration % updateFreqPositionGlob == 0)) {
            positionPID.setSetpoint(positionSetpoint);
            velocitySetpoint = positionPID.update(cumAngle, (float)updateFreqPositionGlob);

            float deltaOffset = positionSetpoint - cumAngle;
        }

        if ((drivingModeGlob > 1) && (iteration % updateFreqVelocityGlob == 0)) {
            velocityPID.setSetpoint(velocitySetpoint);
            torqueSetpoint = velocityPID.update(velocity, (float)updateFreqVelocityGlob);
        }

        if ((drivingModeGlob > 0) && (iteration % updateFreqTorqueGlob == 0)) {
            strength = torqueSetpoint;
            // TODO: read current, and send this to
            // torque PI.
        }

        strength = std::max(-0.8f, std::min(0.8f, strength));
        sumStrength += strength;
        strengthOut = strengthOut * (1 - strengthFilterAlpha) + strength * strengthFilterAlpha;

        float posDelta = PI_DIV_2;
        output.phaseA = strengthOut * sin(elPos + PI_DIV_2 + posDelta);
        output.phaseB = strengthOut * sin(elPos + PI_7_DIV_6 + posDelta);
        output.phaseC = strengthOut * sin(elPos - PI_DIV_6 + posDelta);

        auto err = mcpwm.set_phase_voltages(output.phaseA, output.phaseB, output.phaseC);
    
        numLoops++;
        if (numLoops == 100) {
            if (drivingModeGlob == DrivingMode::Position) {
                positionSetpoint = atomic_load_float(&positionSetpointGlob);
            } else if (drivingModeGlob == DrivingMode::Velocity) {
                velocitySetpoint = atomic_load_float(&velocitySetpointGlob);
            } else if (drivingModeGlob == DrivingMode::Torque) {
                torqueSetpoint = atomic_load_float(&torqueSetpointGlob);
            } else {
                positionSetpoint = 0.0f;
                velocitySetpoint = 0.0f;
                torqueSetpoint = 0.0f;
            }

            atomic_store_float(&angleGlob, angle);
            atomic_store_float(&cumAngleGlob, cumAngle);

            int64_t endTime = esp_timer_get_time();
            auto sumLoopTime = endTime - startTime;
            startTime = endTime;
            float avgLoopTime = static_cast<float>(sumLoopTime) / static_cast<float>(numLoops);
            float avgVelocity = sumVelocity / static_cast<float>(numLoops);
            float avgStrength = sumStrength / static_cast<float>(numLoops);

            atomic_store_float(&avgLoopTimeGlob, avgLoopTime);
            atomic_store_float(&avgVelocityGlob, avgVelocity);
            atomic_store_float(&avgStrengthGlob, avgStrength);

            sumLoopTime = 0;
            sumVelocity = 0;
            sumStrength = 0;
            numLoops = 0;

            ledState = !ledState;
            gpio_set_level(GPIO_NUM_2, ledState);

            uint16_t address = 0x00;
            uint16_t data = 0x00;
            spiManager.motorDriver.readRegister(address, data);
            // printf("Error register: %i\n", data);
        }
    }

    vTaskDelete(NULL);
}

void setupTask() {
    xTaskCreatePinnedToCore(
        realTimeTask,
        "MotorDriverTask",
        4096,
        NULL,
        10,
        &control_task_handle,
        1
    );
}

extern "C" void app_main(void)
{
    setupTask();
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    SerialCom serialCom{};
    SensorData sensorData{};

    uint32_t loopTimeSerial = 0;

    int32_t currentLimit = 10000; // 1A
    bool state = false;
    int iteration = 0;

    drivingModeGlob = 1;
    atomic_store_float(&torqueSetpointGlob, 0.1f);

    while (1) {

        auto startTime = esp_timer_get_time();
        auto current = i2cManager.getCurrent_mA();
        auto voltage = i2cManager.getBusVoltage_mV();
        voltageGlob = voltage;

        iteration++;

        state = !state;
        i2cManager.writePin(MULTIPLEXER_LED0, drivingModeGlob == 1);
        i2cManager.writePin(MULTIPLEXER_LED1, state);

        if (current > currentLimit) {
            // Disable if current is too high.
            drivingModeGlob = DrivingMode::Disabled;
            printf("Disabling motor!\n");
        }

        float velocity = atomic_load_float(&avgVelocityGlob);
        float torque = atomic_load_float(&avgStrengthGlob);
        float looptime = atomic_load_float(&avgLoopTimeGlob);

        sensorData.iteration = iteration;
        sensorData.timestamp_ms = esp_timer_get_time();
        sensorData.position = atomic_load_float(&cumAngleGlob);
        sensorData.velocity = velocity;
        sensorData.torque = torque;
        sensorData.current = current;
        sensorData.voltage = voltage;
        sensorData.loopTimeSerial = loopTimeSerial;
        sensorData.loopTimeMotor = looptime;
        serialCom.setData(sensorData);

        Command cmd{};
        bool gotData = serialCom.getData(cmd);
        if (gotData) {
            switch (cmd.command_type)
            {

            case 1:
                atomic_store_float(&torqueSetpointGlob, cmd.value1);
                break;

            case 2:
                torquePI.setKp(cmd.value1);
                break;

            case 3:
                torquePI.setKi(cmd.value1);
                break;

            case 4:
                // No Command
                break;

            case 5:
                atomic_store_float(&velocitySetpointGlob, cmd.value1);
                break;

            case 6:
                velocityPID.setKp(cmd.value1);
                break;

            case 7:
                velocityPID.setKi(cmd.value1);
                break;

            case 8:
                velocityPID.setKd(cmd.value1);
                break;

            case 9:
                atomic_store_float(&positionSetpointGlob, cmd.value1);
                break;

            case 10:
                positionPID.setKp(cmd.value1);
                break;

            case 11:
                positionPID.setKi(cmd.value1);
                break;

            case 12:
                positionPID.setKd(cmd.value1);
                break;

            case 13:
                drivingModeGlob = cmd.value0;
                break;

            case 14:
                currentLimit = cmd.value0;

            default:
                break;
            }
        }

        if (secondaryTaskStarted.load()) {
            BaseType_t high_task_woken = pdFALSE;
            TaskHandle_t task = static_cast<TaskHandle_t>(oneshotADC.taskHandle);
            vTaskNotifyGiveFromISR(task, &high_task_woken);

            int vA = oneshotADC.getA();
            int vB = oneshotADC.getB();
            int vC = oneshotADC.getC();
            printf("Voltages: %i, %i, %i | %li mV | %li mA\n", vA, vB, vC, voltage, current);
        } else {
            printf("Not startet yet. Bus voltage: %li\n", voltage);
        }
        

        // serialCom.update();
        auto endTime = esp_timer_get_time();
        loopTimeSerial = endTime - startTime;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}