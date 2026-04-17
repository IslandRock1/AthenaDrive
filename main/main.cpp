#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "ContinuousADC.hpp"
#include "Controller.hpp"
#include "PID.hpp"

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

void realTimeTask() {
    
}

extern "C" void app_main(void)
{
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(MULTIPLEXER_MOTOR_ENABLE, true);
    i2cManager.writePin(MULTIPLEXER_MOTOR_CALIBRATION, true);

    printf("Starting when getting power.\n");
    while (i2cManager.getBusVoltage_mV() < 9000) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // IT'S OVER NINE THOUSAND!!
    printf("Starting in 1 second.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    setupLowPins();
    Mcpwm mcpwm = pwm_stuff();
    spiOut spiOutput = spi_stuff();
    setPWM(mcpwm, "0");
    enableLowPins();

    int32_t rotations = 0;
    float angle = 0.0;
    float cumAngle = 0.0;
    float velocity = 0.0;

    ControllerParams params{};
    params.dRegKp = 0.1;
    params.dRegKi = 0.0;
    params.qRegKp = 0.1;
    params.qRegKi = 0.0;
    params.maxD = 1.0;
    params.maxQ = 1.0;
    Controller controller{params};

    mcpwm.set_phase_voltages(0.56, 0.47, 0.47);
    vTaskDelay(pdMS_TO_TICKS(1000));
    float numReadings = 0.0f;
    float sinSum = 0.0f;
    float cosSum = 0.0f;

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

        // printf("Current: %li\n", current);
    }
    float angleOffset = atan2(sinSum, cosSum);

    PID_Reg velocityPID{0.005, 0.0000005, 0.0};
    velocityPID.setSetpoint(20.0);

    int64_t sumLoopTime = 0;
    float sumVelocity = 0;
    float sumStrenght = 0;
    int64_t numLoops = 0;
    while (1) {

        int64_t startTime = esp_timer_get_time();

        // state = !state;
        // i2cManager.writePin(MULTIPLEXER_LED0, state);
        // i2cManager.writePin(MULTIPLEXER_LED1, !state);
        // auto current = i2cManager.getCurrent_mA();

        spiOutput.encoder->update(rotations, angle, cumAngle, velocity);
        sumVelocity += velocity;
        angle -= angleOffset;
        if (angle < 0.0) { angle += TWO_PI; }
        float elPos = fmodf((angle * 20.0f), TWO_PI);
        Output output{};

        float strenght = velocityPID.update(velocity, 1.0);
        sumStrenght += strenght;
        float posDelta = PI_DIV_2;
        output.phaseA = strenght * sin(elPos + PI_DIV_2 + posDelta);
        output.phaseB = strenght * sin(elPos + PI_7_DIV_6 + posDelta);
        output.phaseC = strenght * sin(elPos - PI_DIV_6 + posDelta);

        // printf("%lli,%f,%f,%f,%f,%li,%f\n", startTime, elPos, 0.5 + output.phaseA, 0.5 + output.phaseB, 0.5 + output.phaseC, current, angle);
        auto err = mcpwm.set_phase_voltages(0.5 + output.phaseA, 0.5 + output.phaseB, 0.5 + output.phaseC);
        if (err != ESP_OK) {
            printf("ERROR!");
        }
        
        int64_t endTime = esp_timer_get_time();
        sumLoopTime += (endTime - startTime);
        numLoops++;
        
        if (numLoops == 1000) {
            float avgLoopTime = static_cast<float>(sumLoopTime) / static_cast<float>(numLoops);
            float avgVelocity = sumVelocity / static_cast<float>(numLoops);
            float avgStrenght = sumStrenght / static_cast<float>(numLoops);
            printf("Avg loop time: %f us | Motor speed: %f | Strenght: %f\n", avgLoopTime, avgVelocity, avgStrenght);

            sumLoopTime = 0;
            sumVelocity = 0;
            sumStrenght = 0;
            numLoops = 0;
        }
    }
}