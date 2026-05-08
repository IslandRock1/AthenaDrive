#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "Pinout.hpp"
#include "I2CManager.hpp"
#include "OneshotADC.hpp"
#include "Controller.hpp"
#include "PID.hpp"
#include "SerialComm.hpp"
#include "LowpassFilter.hpp"
#include "GlobalVariableManager.hpp"
#include "RealTimeTask.hpp"

#include "SpiManagerPrimary.hpp"
#include "MCPWM.hpp"

void setupTask() {
    // MotorTaskConfig* config = new MotorTaskConfig{
    //     .MOSI = SPI_MOSI_0,
    //     .MISO = SPI_MISO_0,
    //     .CLK = SPI_CLK_0,
    //     .CHIP_SELECT_ENCODER = CHIP_SELECT_ENCODER,
    //     .CHIP_SELECT_MOTORDRIVER = CHIP_SELECT_MOTOR_DRIVER,
    //     .MOTOR_LOW_A = MOTOR_LOW_A,
    //     .MOTOR_LOW_B = MOTOR_LOW_B,
    //     .MOTOR_LOW_C = MOTOR_LOW_C,
    //     .MOTOR_HIGH_A = MOTOR_HIGH_A,
    //     .MOTOR_HIGH_B = MOTOR_HIGH_B,
    //     .MOTOR_HIGH_C = MOTOR_HIGH_C,
    // };

    // xTaskCreatePinnedToCore(
    //     realTimeTask,
    //     "MotorDriverTask",
    //     4096,
    //     config,
    //     10,
    //     &control_task_handle,
    //     1
    // );
}

extern "C" void app_main(void)
{

    gpio_set_direction(CHIP_SELECT_SD, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_1, GPIO_MODE_OUTPUT);

    gpio_set_level(CHIP_SELECT_SD, true);
    gpio_set_level(CHIP_SELECT_W5500_0, true);
    gpio_set_level(CHIP_SELECT_W5500_1, true);

    uint8_t mode = 0;
    uint32_t numData = 58;
    spi_device_handle_t spiDevice = nullptr;

    spi_bus_config_t busCfg = {
        .mosi_io_num   = SPI_MOSI_1,
        .miso_io_num   = SPI_MISO_1,
        .sclk_io_num   = SPI_CLK_1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = mode,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = 1000000,
        .spics_io_num = CHIP_SELECT_W5500_0,
        .queue_size = 1,
    };

    spi_bus_add_device(SPI2_HOST, &devConfig, &spiDevice);

    uint8_t txBuffer[3 + numData] = { 0 };
    uint8_t rxBuffer[3 + numData] = { 0 };

    txBuffer[0] = 0x0000; // Upper Address
    txBuffer[1] = 0x0000; // Lower Address
    txBuffer[2] = 0b00000000; // 5x BSB, 1x RW, 2x OM

    spi_transaction_t spiTransaction = {
        .length = 16 + 8 + 8 * numData, // Address + Control + byte x numData 
        .tx_buffer = txBuffer,
        .rx_buffer = rxBuffer,
    };

    err = spi_device_transmit(spiDevice, &spiTransaction);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }

    printf("Recieving\n");
    for (uint32_t i = 0; i < numData; i++) {
        printf("Addr: 0x%02lX | Data: %i\n", i, rxBuffer[i + 3]);
    }
    printf("\n");

    return;

    setupTask();
    I2CManager i2cManager{I2C_SDA, I2C_SCL};
    i2cManager.writePin(IO_EXPANDER_MOTOR_ENABLE, true);
    i2cManager.writePin(IO_EXPANDER_MOTOR_CALIBRATION, false);

    SerialCom serialCom{};
    SensorData sensorData{};

    uint32_t loopTimeSerial = 0;

    int32_t currentLimit = 10000;
    bool state = false;
    int iteration = 0;

    LowpassFilter lowpassCurrent{0.1f};
    LowpassFilter lowpassVoltage{0.1f};

    while (1) {
        auto startTime = esp_timer_get_time();

        if (globalVariableManager.getWantedCalibrationMode()) {
            i2cManager.writePin(IO_EXPANDER_MOTOR_CALIBRATION, true);
            globalVariableManager.setActualCalibrationMode(true);
        } else {
            i2cManager.writePin(IO_EXPANDER_MOTOR_CALIBRATION, false);
            globalVariableManager.setActualCalibrationMode(false);
        }

        auto current = lowpassCurrent.update(i2cManager.getCurrent_mA());
        auto voltage = lowpassVoltage.update(i2cManager.getBusVoltage_mV());
        globalVariableManager.setVoltage(voltage);

        iteration++;

        state = !state;
        i2cManager.writePin(IO_EXPANDER_LED0, globalVariableManager.getDrivingMode() == 1);
        i2cManager.writePin(IO_EXPANDER_LED1, state);

        if (current > currentLimit) {
            // Disable if current is too high.
            globalVariableManager.setDrivingMode(DrivingMode::Disabled);
        }

        float velocity = globalVariableManager.getAvgVelocity();
        float torque = globalVariableManager.getAvgStrength();
        float looptime = globalVariableManager.getAvgLoopTime();

        sensorData.iteration = iteration;
        sensorData.timestamp_ms = esp_timer_get_time();
        sensorData.position = globalVariableManager.getCumAngle();
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
                globalVariableManager.setTorqueSetpoint(cmd.value1);
                break;

            case 2:
                globalVariableManager.setTorqueKp(cmd.value1);
                break;

            case 3:
                globalVariableManager.setTorqueKi(cmd.value1);
                break;

            case 4:
                // No Command
                break;

            case 5:
                globalVariableManager.setVelocitySetpoint(cmd.value1);
                break;

            case 6:
                globalVariableManager.setVelocityKp(cmd.value1);
                break;

            case 7:
                globalVariableManager.setVelocityKi(cmd.value1);
                break;

            case 8:
                globalVariableManager.setVelocityKd(cmd.value1);
                break;

            case 9:
                globalVariableManager.setPositionSetpoint(cmd.value1);
                break;

            case 10:
                globalVariableManager.setPositionKp(cmd.value1);
                break;

            case 11:
                globalVariableManager.setPositionKi(cmd.value1);
                break;

            case 12:
                globalVariableManager.setPositionKd(cmd.value1);
                break;

            case 13:
                globalVariableManager.setDrivingMode(cmd.value0);
                break;

            case 14:
                currentLimit = cmd.value0;
                break;

            case 15:
                globalVariableManager.setNumPolePairs(cmd.value0);
                break;
            
            case 16:
                globalVariableManager.setOpenLoopSpeed(cmd.value1);
                break;
            
            case 17:
                globalVariableManager.setOpenLoopStrength(cmd.value1);
                break;

            default:
                break;
            }
        }

        serialCom.update();
        auto endTime = esp_timer_get_time();
        loopTimeSerial = endTime - startTime;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}