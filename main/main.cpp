#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdatomic.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "ContinuousADC.hpp"
#include "Controller.hpp"
#include "PID.hpp"
#include "SerialComm.hpp"

#include "DRV8323.hpp"
#include "DRV8323_Registers.hpp"
#include "AS5048.hpp"
#include "AS5048_Registers.hpp"
#include "MCPWM.hpp"

constexpr float PI_CONST = 3.1415926535897f;
constexpr float TWO_PI = 2.0f * PI_CONST;
constexpr float PI_DIV_2 = PI_CONST/ 2.0f;
constexpr float PI_7_DIV_6 = 7.0f * PI_CONST / 6.0f;
constexpr float PI_DIV_6 = PI_CONST / 6.0f;

struct spiOut {
    DRV8323 *motorDriver;
    AS5048 *encoder;
};
spiOut spiOutput{};

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
        .spiClockHz = 100000,
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
Mcpwm mcpwm;

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

Output setPWM(Mcpwm &mcpwm, std::string direction) {
    float vA = 0.5;
    float vB = 0.5;
    float vC = 0.5;
    float size = 0.050;
    
    if (direction == "A+") {
        vA += size;
        vB -= size;
        vC -= size;
    } else if (direction == "A-") {
        vA -= size;
        vB += size;
        vC += size;
    } else if (direction == "B+") {
        vA -= size;
        vB += size;
        vC -= size;
    } else if (direction == "B-") {
        vA += size;
        vB -= size;
        vC += size;
    } else if (direction == "C+") {
        vA -= size;
        vB -= size;
        vC += size;
    } else if (direction == "C-") {
        vA += size;
        vB += size;
        vC -= size;
    }

    mcpwm.set_phase_voltages(vA, vB, vC);
    return {vA, vB, vC};
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

PID_Reg velocityPID{0.005, 0.0, 0.0};
atomic_uint velocitySetpointGlob = 0.0;

void realTimeTask(void *pvParameters) {

    while (1) {
        int64_t startTime = esp_timer_get_time();

        static int32_t rotations;
        static float angle, cumAngle, velocity;

        static float sumVelocity = 0;
        static float sumStrength = 0;
        static float sumLoopTime = 0;
        static float numLoops = 0;

        spiOutput.encoder->update(rotations, angle, cumAngle, velocity);
        rotationsGlob = rotations;
        atomic_store_float(&angleGlob, angle);
        atomic_store_float(&cumAngleGlob, cumAngle);

        sumVelocity += velocity;
        angle -= angleOffset;
        if (angle < 0.0) { angle += TWO_PI; }
        float elPos = fmodf((angle * 20.0f), TWO_PI);
        Output output{};

        float velSetpoint = atomic_load_float(&velocitySetpointGlob);
        velocityPID.setSetpoint(velSetpoint);
        float strenght = velocityPID.update(velocity, 1.0);
        strenght = std::max(-0.4f, std::min(0.4f, strenght));

        sumStrength += strenght;
        float posDelta = PI_DIV_2;
        output.phaseA = strenght * sin(elPos + PI_DIV_2 + posDelta);
        output.phaseB = strenght * sin(elPos + PI_7_DIV_6 + posDelta);
        output.phaseC = strenght * sin(elPos - PI_DIV_6 + posDelta);

        auto err = mcpwm.set_phase_voltages(0.5 + output.phaseA, 0.5 + output.phaseB, 0.5 + output.phaseC);
        
        int64_t endTime = esp_timer_get_time();
        sumLoopTime += (endTime - startTime);
        numLoops++;
        
        if (numLoops == 1000) {
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

extern "C" void app_main(void)
{

    mcpwm = pwm_stuff();
    mcpwm.set_phase_voltages(0.5f, 0.5f, 0.5f);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return;

    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    printf("Starting when getting power.\n");
    while (i2cManager.getBusVoltage_mV() < 9000) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // IT'S OVER NINE THOUSAND!!
    printf("Voltage: %li\n", i2cManager.getBusVoltage_mV());
    printf("Starting in 1 second.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    setupLowPins();
    mcpwm = pwm_stuff();
    spiOutput = spi_stuff();
    setPWM(mcpwm, "0");
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

        auto current = i2cManager.getCurrent_mA();
        if (current > 1000) {
            mcpwm.set_phase_voltages(0.5, 0.5, 0.5);
            printf("Too much current: %li\n", current);
            return;
        }
    }
    angleOffset = atan2(sinSum, cosSum);

    xTaskCreatePinnedToCore(
        realTimeTask,
        "MotorDriverTask",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    SerialCom serialCom{};
    SensorData sensorData{};

    bool state = false;
    int iteration = 0;
    while (1) {
        iteration++;
        state = !state;
        i2cManager.writePin(MULTIPLEXER_LED0, state);
        i2cManager.writePin(MULTIPLEXER_LED1, !state);
        auto current = i2cManager.getCurrent_mA();
        
        float velocity = atomic_load_float(&avgVelocityGlob);
        float torque = atomic_load_float(&avgStrenghtGlob);
        float looptime = atomic_load_float(&avgLoopTimeGlob);

        sensorData.iteration = iteration;
        sensorData.timestamp_ms = esp_timer_get_time();
        sensorData.position = atomic_load_float(&cumAngleGlob); // static_cast<float>(rotationsGlob) * TWO_PI + atomic_load_float(&angleGlob);
        sensorData.velocity = velocity;
        sensorData.torque = torque;
        sensorData.current = current;
        serialCom.setData(sensorData);

        Command cmd{};
        bool gotData = serialCom.getData(cmd);
        if (gotData) {
            switch (cmd.command_type)
            {
            case 1:
                atomic_store_float(&velocitySetpointGlob, cmd.value1);
                break;
            
            case 2:
                velocityPID.setKp(cmd.value1);
                break;
            
            case 3:
                velocityPID.setKi(cmd.value1);
                break;
            
            case 4:
                velocityPID.setKd(cmd.value1);
                break;

            default:
                break;
            }
        }

        serialCom.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}