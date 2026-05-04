#include <stdio.h>
#include "RealTimeTask.hpp"

gptimer_handle_t timer = NULL;
TaskHandle_t control_task_handle = NULL;
OneshotADC oneshotADC;

void setupSPI(SpiManagerPrimary &spiManager, MotorTaskConfig config) {
    SpiConfigPrimary configPrimary = {
        .MOSI = config.MOSI,
        .MISO = config.MISO,
        .CLK = config.CLK,
        .SPI_HOST = SPI2_HOST,
    };

    spiManager.beginManager(configPrimary);
}

void beginEncoder(SpiManagerPrimary &spiManager, MotorTaskConfig config) {
    EncoderConfig configEnc(SPI2_HOST, config.CHIP_SELECT_ENCODER, 10000000, 3);
    spiManager.beginEncoder(configEnc);
}

void beginMotorDriver(SpiManagerPrimary &spiManager, MotorTaskConfig config) {
    MotorDriverConfig configDrv(SPI2_HOST, config.CHIP_SELECT_MOTORDRIVER, 100000, 1, config.MOTOR_LOW_A, config.MOTOR_LOW_B, config.MOTOR_LOW_C);
    spiManager.beginMotorDriver(configDrv);
}

void pwm_stuff(Mcpwm &mcpwm, MotorTaskConfig config) {
    Mcpwm::Config pwmCfg = {};
    pwmCfg.pwm_a_gpio = config.MOTOR_HIGH_A;
    pwmCfg.pwm_b_gpio = config.MOTOR_HIGH_B;
    pwmCfg.pwm_c_gpio = config.MOTOR_HIGH_C;
    pwmCfg.n_sleep_gpio = GPIO_NUM_NC;

    mcpwm.init(pwmCfg);
    oneshotADC.startTask();
    mcpwm.register_adc_trigger(oneshotADC.taskHandle);
    mcpwm.enable();
    mcpwm.set_phase_voltages(-1.0, -1.0, -1.0);
}
Mcpwm mcpwm;

static bool IRAM_ATTR timerCallback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t task = (TaskHandle_t) user_ctx;
    vTaskNotifyGiveFromISR(task, &high_task_woken);
    return (high_task_woken == pdTRUE);
}

void initTimer(TaskHandle_t task_handle)
{
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz = 1 tick per µs
    };

    gptimer_new_timer(&config, &timer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timerCallback,
    };

    gptimer_register_event_callbacks(timer, &cbs, task_handle);
    gptimer_enable(timer);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 300,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_set_alarm_action(timer, &alarm_config);
    gptimer_start(timer);
}

void IRAM_ATTR realTimeTask(void *pvParameters) {
    MotorTaskConfig* config = static_cast<MotorTaskConfig*>(pvParameters);
    SpiManagerPrimary spiManager;
    Mcpwm mcpwm;

    setupSPI(spiManager, *config);
    beginEncoder(spiManager, *config);
    ControllerParams controllerParams{0.01, 0.0, 0.01, 0.0, 0.0, 0.0};
    Controller controller{controllerParams};

    float angleOffset = 0;
    PI_Reg torquePI{0.0, 0.0};
    PID_Reg velocityPID{0.0, 0.0, 0.0};
    PID_Reg positionPID{0.0, 0.0, 0.0};

    int32_t rotations;
    float angle, cumAngle, velocity;
    while (globalVariableManager.getVoltage() < 9000) {
        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        globalVariableManager.setCumAngle(cumAngle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Wait 1 second.
    for (int i = 0; i < 100; i++) {
        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        globalVariableManager.setCumAngle(cumAngle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    beginMotorDriver(spiManager, *config);

    pwm_stuff(mcpwm, *config);
    mcpwm.set_phase_voltages(0.0, 0.0, 0.0);
    spiManager.motorDriver.enable();

    globalVariableManager.setWantedCalibrationMode(true);
    while (!globalVariableManager.getActualCalibrationMode()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int sumAdcA = 0;
    int sumAdcB = 0;
    int sumAdcC = 0;

    for (int i = 0; i < 10; i++) {
        sumAdcA += oneshotADC.getA();
        sumAdcB += oneshotADC.getB();
        sumAdcC += oneshotADC.getC();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float baselineAdcA = static_cast<float>(sumAdcA) / 10000.0;
    float baselineAdcB = static_cast<float>(sumAdcB) / 10000.0;
    float baselineAdcC = static_cast<float>(sumAdcC) / 10000.0;
    globalVariableManager.setWantedCalibrationMode(false);

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
    // Needs to add sign change based on velocity after angle offset.
    // Sometimes positive torque => positive angle/velocity
    // Other times positive torque => negative
    // Needs to offset angleOffset by PI/2 based on sign.

    LowpassFilter lowpassVelocity{0.001};
    LowpassFilter lowpassStrength{0.001};
    LowpassFilter lowpassLoopTime{0.001};
    LowpassFilter lowpassStrengthOut{0.001};
    static float numLoops = 0;

    float positionSetpoint = globalVariableManager.getPositionSetpoint();
    float velocitySetpoint = globalVariableManager.getVelocitySetpoint();
    float torqueSetpoint = globalVariableManager.getTorqueSetpoint();

    uint32_t drivingMode = globalVariableManager.getDrivingMode();
    uint32_t updateFreqPos = globalVariableManager.getUpdateFreqPosition();
    uint32_t updateFreqVel = globalVariableManager.getUpdateFreqVelocity();
    uint32_t updateFreqTor = globalVariableManager.getUpdateFreqTorque();

    float numPolePairs = globalVariableManager.getNumPolePairs();

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, true);
    bool ledState = true;

    uint64_t iteration = 0;
    initTimer(control_task_handle);
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t startTime = esp_timer_get_time();
        iteration++;

        spiManager.encoder.update(rotations, angle, cumAngle, velocity);
        angle -= angleOffset;
        rotations *= -1;
        angle *= -1.0f;
        cumAngle *= -1.0f;
        velocity *= -1.0f;

        globalVariableManager.setRotations(rotations);
        lowpassVelocity.update(velocity);
        
        float elPos = angle * numPolePairs;
        while (elPos >= GlobalVariableManager::TWO_PI) { elPos -= GlobalVariableManager::TWO_PI; }
        while (elPos < 0) { elPos += GlobalVariableManager::TWO_PI; }

        float deltaOffset = 0.0f;
        float strength = 0.0f;

        if ((drivingMode > 2) && (iteration % updateFreqPos == 0)) {
            positionPID.setSetpoint(positionSetpoint);
            velocitySetpoint = positionPID.update(cumAngle, static_cast<float>(updateFreqPos));

            float deltaOffset = positionSetpoint - cumAngle;
        }

        if ((drivingMode > 1) && (iteration % updateFreqVel == 0)) {
            velocityPID.setSetpoint(velocitySetpoint);
            torqueSetpoint = velocityPID.update(velocity, static_cast<float>(updateFreqVel));
        }

        if ((drivingMode > 0) && (iteration % updateFreqTor == 0)) {
            strength = torqueSetpoint;
        }

        if (drivingMode == 0) {
            strength = 0.0f;
        }

        float strengthOut = lowpassStrengthOut.update(strength);
        float Ia = (static_cast<float>(oneshotADC.getA()) / 1000.0f - baselineAdcA) / (40.0f * 0.0035);
        float Ib = (static_cast<float>(oneshotADC.getB()) / 1000.0f - baselineAdcB) / (40.0f * 0.0035);
        float Ic = (static_cast<float>(oneshotADC.getC()) / 1000.0f - baselineAdcC) / (40.0f * 0.0035);
        // If using Ia and Ib in controller, it starts shaking like crazy.
        // I am 99% sure there is something wrong with the resulting current.
        // Either analog reading, current calculations, calibration.. idk.

        lowpassStrength.update(strengthOut);

        // Using strengthOut to get positive strength to line up with positive position/velocity.
        Output output = controller.update(strengthOut, elPos, velocity, 0.0, 0.0);
        float maxOut = std::max(std::abs(output.phaseA), std::max(std::abs(output.phaseB), std::abs(output.phaseC)));
        if (maxOut > 0.8) {
            float k = 0.8 / maxOut;
            output.phaseA *= k;
            output.phaseB *= k;
            output.phaseC *= k;
        }
        auto err = mcpwm.set_phase_voltages(output.phaseA, output.phaseB, output.phaseC);
    
        int64_t endTime = esp_timer_get_time();
        lowpassLoopTime.update(endTime - startTime);

        numLoops++;
        if (numLoops == 100) {
            drivingMode = globalVariableManager.getDrivingMode();
            if (drivingMode == DrivingMode::Position) {
                positionSetpoint = globalVariableManager.getPositionSetpoint();
            } else if (drivingMode == DrivingMode::Velocity) {
                velocitySetpoint = globalVariableManager.getVelocitySetpoint();
            } else if (drivingMode == DrivingMode::Torque) {
                torqueSetpoint = globalVariableManager.getTorqueSetpoint();
            } else {
                positionSetpoint = 0.0f;
                velocitySetpoint = 0.0f;
                torqueSetpoint = 0.0f;
            }

            positionPID.setKp(globalVariableManager.getPositionKp());
            positionPID.setKi(globalVariableManager.getPositionKi());
            positionPID.setKd(globalVariableManager.getPositionKd());
            velocityPID.setKp(globalVariableManager.getVelocityKp());
            velocityPID.setKi(globalVariableManager.getVelocityKi());
            velocityPID.setKd(globalVariableManager.getVelocityKd());

            globalVariableManager.setCumAngle(cumAngle);
            numPolePairs = globalVariableManager.getNumPolePairs();

            float floatingNumLoops = static_cast<float>(numLoops);
            float avgLoopTime = lowpassLoopTime.getValue();
            float avgVelocity = lowpassVelocity.getValue();
            float avgStrength = lowpassStrength.getValue();

            globalVariableManager.setAvgLooptime(avgLoopTime);
            globalVariableManager.setAvgVelocity(avgVelocity);
            globalVariableManager.setAvgStrength(avgStrength);
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