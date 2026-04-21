#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdatomic.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "ContinuousADC.hpp"
#include "PID.hpp"
#include "PI.hpp"
#include "SerialComm.hpp"

#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"
#include "AS5048.hpp"
#include "AS5048_Registers.hpp"
#include "MCPWM.hpp"

struct Output {
	float phaseA;
	float phaseB;
	float phaseC;
};

constexpr float PI_CONST = 3.1415926535897f;
constexpr float TWO_PI = 2.0f * PI_CONST;
constexpr float PI_DIV_2 = PI_CONST/ 2.0f;
constexpr float PI_7_DIV_6 = 7.0f * PI_CONST / 6.0f;
constexpr float PI_DIV_6 = PI_CONST / 6.0f;

struct spiOut {
    DRV8323 *motorDriver;
    AS5048 *encoder;
};

spiOut spi_stuff() {
    spi_bus_config_t busCfg = {
        .mosi_io_num   = SPI_MOSI_0,
        .miso_io_num   = SPI_MISO_0,
        .sclk_io_num   = SPI_CLK_0,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    drvConfig configDrv = {
        .spiHost = SPI2_HOST,
        .cs = CHIP_SELECT_MOTOR_DRIVER,
        .spiClockHz = 100000,
    };
    auto motorDriver = new DRV8323(configDrv);

    encoderConfig configEnc = {
        .spiHost = SPI2_HOST,
        .cs = CHIP_SELECT_ENCODER,
        .spiClockHz = 10 * 000000,
    };
    auto encoder = new AS5048(configEnc);

    uint16_t error = 0;
    encoder->readRegister(AS5048_REG_CLEAR_ERROR, &error);

    vTaskDelay(pdMS_TO_TICKS(10));
    motorDriver->modifyBits(DRV_REG_CSA_CTRL, DRV_CSA_GAIN_MASK, DRV_CSA_GAIN_40);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t val = 0;
    motorDriver->readRegister(0x02, &val);

    val &= ~(0b1100000);     // clear PWM_MODE
    val |=  (0b0100000);      // set 3x PWM

    motorDriver->writeRegister(0x02, val);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t mdrvData = 0;
    motorDriver->readRegister(0x02, &mdrvData);
    printf("Got %i from motordriver.\n", mdrvData);
    motorDriver->readRegister(0x06, &mdrvData);
    printf("Got %i from motordriver.\n", mdrvData);
    vTaskDelay(pdMS_TO_TICKS(3 * 1000));

    spiOut out = spiOut{motorDriver, encoder};
    return out;
}

Mcpwm pwm_stuff() {
    Mcpwm::Config pwmCfg = {};
    pwmCfg.pwm_a_gpio = MOTOR_HIGH_A;
    pwmCfg.pwm_b_gpio = MOTOR_HIGH_B;
    pwmCfg.pwm_c_gpio = MOTOR_HIGH_C;
    pwmCfg.n_sleep_gpio = GPIO_NUM_NC;

    Mcpwm mcpwm;
    mcpwm.init(pwmCfg);
    mcpwm.enable();
    mcpwm.set_phase_voltages(-1.0, -1.0, -1.0);

    return mcpwm;
}

void setupLowPins() {
    gpio_set_direction(MOTOR_LOW_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LOW_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LOW_C, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_LOW_A, false);
    gpio_set_level(MOTOR_LOW_B, false);
    gpio_set_level(MOTOR_LOW_C, false);
}

void enableLowPins() {
    gpio_set_level(MOTOR_LOW_A, true);
    gpio_set_level(MOTOR_LOW_B, true);
    gpio_set_level(MOTOR_LOW_C, true);
}

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

atomic_int32_t rotationsGlob = 0;
atomic_uint angleGlob = 0.0;
atomic_uint cumAngleGlob = 0.0;
atomic_uint velocityGlob = 0.0;

atomic_uint avgVelocityGlob = 0;
atomic_uint avgStrenghtGlob = 0;
atomic_uint avgLoopTimeGlob = 0;

float angleOffset = 0;

atomic_uint drivingModeGlob = 0;
/*
drivingMode =>
0: Disabled
1: Torque
2: Velocity
3: Position
*/

PI_Reg torquePI{0.0, 0.0};
atomic_uint torqueSetpointGlob = 0.0;
atomic_uint updateFreqTorqueGlob = 1; // TODO!

PID_Reg velocityPID{0.0, 0.0, 0.0};
atomic_uint velocitySetpointGlob = 0.0;
atomic_uint updateFreqVelocityGlob = 10; // TODO!

PID_Reg positionPID{0.0, 0.0, 0.0};
atomic_uint positionSetpointGlob = 0.0;
atomic_uint updateFreqPositionGlob = 100; // TODO!

void IRAM_ATTR realTimeTask(void *pvParameters) {

    setupLowPins();
    Mcpwm mcpwm = pwm_stuff();
    spiOut spiOutput = spi_stuff();
    mcpwm.set_phase_voltages(0.5, 0.5, 0.5);
    enableLowPins();

    mcpwm.set_phase_voltages(0.56, 0.47, 0.47);
    vTaskDelay(pdMS_TO_TICKS(1000));
    float numReadings = 0.0f;
    float sinSum = 0.0f;
    float cosSum = 0.0f;

    int32_t rotations;
    float angle, cumAngle, velocity;

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        spiOutput.encoder->update(rotations, angle, cumAngle, velocity);
        numReadings += 1.0;

        sinSum += sin(angle);
        cosSum += cos(angle);
    }
    angleOffset = atan2(sinSum, cosSum);

    static float sumVelocity = 0;
    static float sumStrength = 0;
    static float sumLoopTime = 0;
    static float numLoops = 0;

    float positionSetpoint = atomic_load_float(&positionSetpointGlob);
    float velocitySetpoint = atomic_load_float(&velocitySetpointGlob);
    float torqueSetpoint = atomic_load_float(&torqueSetpointGlob);

    float strengthOut = 0.0f;
    float strengthFilterAlpha = 0.001f;

    uint64_t iteration = 0;
    int64_t startTime = esp_timer_get_time();
    while (1) {
        iteration++;

        spiOutput.encoder->update(rotations, angle, cumAngle, velocity);
        rotationsGlob = rotations;

        sumVelocity += velocity;
        angle -= angleOffset;
        if (angle < 0.0) { angle += TWO_PI; }

        // float elPos = fmodf((angle * 20.0f), TWO_PI);
        float temp = angle * 7.0f;
        while (temp >= TWO_PI) temp -= TWO_PI;
        while (temp < 0) temp += TWO_PI;
        float elPos = temp;

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

        strength = std::max(-0.4f, std::min(0.4f, strength));
        sumStrength += strength;
        strengthOut = strengthOut * (1 - strengthFilterAlpha) + strength * strengthFilterAlpha;

        float posDelta;
        if (strength > 0) {
            posDelta = PI_DIV_2;
        } else {
            posDelta = -PI_DIV_2;
        }
        
        posDelta = PI_DIV_2;
        output.phaseA = strengthOut * sin(elPos + PI_DIV_2 + posDelta);
        output.phaseB = strengthOut * sin(elPos + PI_7_DIV_6 + posDelta);
        output.phaseC = strengthOut * sin(elPos - PI_DIV_6 + posDelta);

        auto err = mcpwm.set_phase_voltages(0.5 + output.phaseA, 0.5 + output.phaseB, 0.5 + output.phaseC);
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
            float avgStrenght = sumStrength / static_cast<float>(numLoops);

            atomic_store_float(&avgLoopTimeGlob, avgLoopTime);
            atomic_store_float(&avgVelocityGlob, avgVelocity);
            atomic_store_float(&avgStrenghtGlob, avgStrenght);

            sumLoopTime = 0;
            sumVelocity = 0;
            sumStrength = 0;
            numLoops = 0;
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
        24,
        NULL,
        1
    );
}

extern "C" void app_main(void)
{
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    bool startedTask = false;
    bool voltageOver9V = false;
    int64_t timeOfPower = 0;
    
    SerialCom serialCom{};
    SensorData sensorData{};

    uint32_t loopTimeSerial = 0;

    int32_t currentLimit = 1000; // 1A
    bool state = false;
    int iteration = 0;
    while (1) {
        auto startTime = esp_timer_get_time();
        auto current = i2cManager.getCurrent_mA();
        auto voltage = i2cManager.getBusVoltage_mV();
        // printf("Voltage: %li\n", voltage);
        if (!startedTask) {
            if ((!voltageOver9V) && (voltage > 9000)) {
                timeOfPower = startTime;
                voltageOver9V = true;
            } else if ((voltageOver9V) && (startTime - timeOfPower > 1000 * 1000)) {
                setupTask();
                startedTask = true;
                // printf("Stared task!\n");
            }
        }

        iteration++;

        state = !state;
        i2cManager.writePin(MULTIPLEXER_LED0, drivingModeGlob == 1);
        i2cManager.writePin(MULTIPLEXER_LED1, state);
        
        if (current > currentLimit) {
            // Disable of current is too high.
            drivingModeGlob = DrivingMode::Disabled;
        }
        
        float velocity = atomic_load_float(&avgVelocityGlob);
        float torque = atomic_load_float(&avgStrenghtGlob);
        float looptime = atomic_load_float(&avgLoopTimeGlob);

        sensorData.iteration = iteration;
        sensorData.timestamp_ms = esp_timer_get_time();
        sensorData.position = atomic_load_float(&cumAngleGlob);
        sensorData.velocity = velocity;
        sensorData.torque = torque;
        sensorData.current = current;
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

        serialCom.update();
        auto endTime = esp_timer_get_time();
        loopTimeSerial = endTime - startTime;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}