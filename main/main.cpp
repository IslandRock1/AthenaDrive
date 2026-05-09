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

static const char* TAG = "W5500";
static spi_device_handle_t spiDevice = nullptr;

// ── W5500 control byte helpers ────────────────────────────────────────────────
// BSB: 00000 = common regs, 00001 = socket regs, 00010 = socket TX, 00011 = socket RX
#define W5500_COMMON_REG  0x00
#define W5500_SKT_REG(n)  ((n) << 5 | 0x08)   // socket n register block
#define W5500_SKT_TX(n)   ((n) << 5 | 0x10)   // socket n TX buffer
#define W5500_SKT_RX(n)   ((n) << 5 | 0x18)   // socket n RX buffer
#define W5500_READ        0x00
#define W5500_WRITE       0x04

// ── Common register addresses ─────────────────────────────────────────────────
#define REG_MR    0x0000   // Mode
#define REG_GAR   0x0001   // Gateway IP (4 bytes)
#define REG_SUBR  0x0005   // Subnet mask (4 bytes)
#define REG_SHAR  0x0009   // MAC address (6 bytes)
#define REG_SIPR  0x000F   // Source IP (4 bytes)

// ── Socket register addresses (relative within socket block) ──────────────────
#define Sn_MR     0x0000   // Socket mode
#define Sn_CR     0x0001   // Socket command
#define Sn_SR     0x0003   // Socket status
#define Sn_PORT   0x0004   // Source port (2 bytes)
#define Sn_RX_RSR 0x0026   // Received byte count (2 bytes)
#define Sn_RX_RD  0x0028   // RX read pointer (2 bytes)

// Socket commands
#define CMD_OPEN  0x01
#define CMD_RECV  0x40

// Socket modes
#define MR_UDP    0x02

// Socket statuses
#define SOCK_UDP  0x22

#define Sn_DIPR   0x000C   // Destination IP (4 bytes)
#define Sn_DPORT  0x0010   // Destination port (2 bytes)
#define Sn_TX_FSR 0x0020   // TX free size (2 bytes)
#define Sn_TX_WR  0x0024   // TX write pointer (2 bytes)
#define CMD_SEND  0x20

// ── Low-level SPI read / write ─────────────────────────────────────────────────
static void w5500_write(uint16_t addr, uint8_t block, const uint8_t* data, size_t len) {
    uint8_t buf[3 + len];
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFF);
    buf[2] = (uint8_t)(block | W5500_WRITE);
    memcpy(buf + 3, data, len);

    spi_transaction_t t = {};
    t.length    = (3 + len) * 8;
    t.tx_buffer = buf;
    spi_device_polling_transmit(spiDevice, &t);
}

static void w5500_read(uint16_t addr, uint8_t block, uint8_t* out, size_t len) {
    uint8_t tx[3 + len];
    uint8_t rx[3 + len];
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = (uint8_t)(addr >> 8);
    tx[1] = (uint8_t)(addr & 0xFF);
    tx[2] = (uint8_t)(block | W5500_READ);

    spi_transaction_t t = {};
    t.length    = (3 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(spiDevice, &t);

    memcpy(out, rx + 3, len);  // skip the 3 header echo bytes
}

// Convenience wrappers for single bytes / 16-bit registers
static void     w5500_write8 (uint16_t a, uint8_t b, uint8_t v)  { w5500_write(a, b, &v, 1); }
static uint8_t  w5500_read8  (uint16_t a, uint8_t b)             { uint8_t v; w5500_read(a, b, &v, 1); return v; }
static void     w5500_write16(uint16_t a, uint8_t b, uint16_t v) { uint8_t buf[2] = {(uint8_t)(v>>8), (uint8_t)v}; w5500_write(a, b, buf, 2); }
static uint16_t w5500_read16 (uint16_t a, uint8_t b)             { uint8_t buf[2]; w5500_read(a, b, buf, 2); return (buf[0]<<8)|buf[1]; }

// ── Network init ───────────────────────────────────────────────────────────────
static void w5500_net_init() {
    // Software reset
    w5500_write8(REG_MR, W5500_COMMON_REG, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t mac[6]     = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
    uint8_t gateway[4] = { 192, 168, 50, 1 };
    uint8_t subnet[4]  = { 255, 255, 255, 0 };
    uint8_t ip[4]      = { 192, 168, 50, 10 };  // ← pick a free IP on your LAN

    w5500_write(REG_SHAR, W5500_COMMON_REG, mac,     6);
    w5500_write(REG_GAR,  W5500_COMMON_REG, gateway, 4);
    w5500_write(REG_SUBR, W5500_COMMON_REG, subnet,  4);
    w5500_write(REG_SIPR, W5500_COMMON_REG, ip,      4);
}

// ── Open socket 0 in UDP mode ─────────────────────────────────────────────────
static void socket_udp_open(uint8_t skt, uint16_t port) {
    w5500_write8 (Sn_MR,   W5500_SKT_REG(skt), MR_UDP);
    w5500_write16(Sn_PORT, W5500_SKT_REG(skt), port);
    w5500_write8 (Sn_CR,   W5500_SKT_REG(skt), CMD_OPEN);

    // Wait until SOCK_UDP
    while (w5500_read8(Sn_SR, W5500_SKT_REG(skt)) != SOCK_UDP) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Socket %d open on UDP port %d", skt, port);
}

static void socket_udp_send(uint8_t skt, uint8_t* destIp, uint16_t destPort,
                             const uint8_t* data, uint16_t len) {
    // Set destination IP and port
    w5500_write(Sn_DIPR,  W5500_SKT_REG(skt), destIp, 4);
    w5500_write16(Sn_DPORT, W5500_SKT_REG(skt), destPort);

    // Wait for TX buffer to have space
    while (w5500_read16(Sn_TX_FSR, W5500_SKT_REG(skt)) < len) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Write data at TX write pointer
    uint16_t ptr = w5500_read16(Sn_TX_WR, W5500_SKT_REG(skt));
    w5500_write(ptr, W5500_SKT_TX(skt), data, len);

    // Advance TX write pointer and trigger send
    w5500_write16(Sn_TX_WR, W5500_SKT_REG(skt), ptr + len);
    w5500_write8 (Sn_CR,    W5500_SKT_REG(skt), CMD_SEND);
}

// ── Poll and print any incoming UDP packet ────────────────────────────────────
// W5500 UDP RX frame: 6 bytes src IP+port, 2 bytes length, then payload
static void socket_udp_recv(uint8_t skt) {
    uint16_t size = w5500_read16(Sn_RX_RSR, W5500_SKT_REG(skt));
    if (size == 0) return;

    uint16_t ptr = w5500_read16(Sn_RX_RD, W5500_SKT_REG(skt));

    // Read the 8-byte header: [4 src IP][2 src port][2 payload len]
    uint8_t header[8];
    w5500_read(ptr, W5500_SKT_RX(skt), header, 8);
    ptr += 8;

    uint8_t  srcIp[4]   = { header[0], header[1], header[2], header[3] };
    uint16_t srcPort     = (header[4] << 8) | header[5];
    uint16_t payloadLen  = (header[6] << 8) | header[7];

    uint8_t payload[payloadLen + 1];
    memset(payload, 0, sizeof(payload));
    w5500_read(ptr, W5500_SKT_RX(skt), payload, payloadLen);
    ptr += payloadLen;

    // Advance RX pointer and issue RECV
    w5500_write16(Sn_RX_RD, W5500_SKT_REG(skt), ptr);
    w5500_write8 (Sn_CR,    W5500_SKT_REG(skt), CMD_RECV);

    // ESP_LOGI(TAG, "UDP from %d.%d.%d.%d:%d  [%d bytes]: %s",
    //     srcIp[0], srcIp[1], srcIp[2], srcIp[3],
    //     srcPort, payloadLen, (char*)payload);

    // ── Do a simple operation: parse int, double it, send it back ────────────
    int value = atoi((char*)payload);
    int result = value * 2;

    char response[32];
    int responseLen = snprintf(response, sizeof(response), "%d * 2 = %d\n", value, result);

    socket_udp_send(skt, srcIp, srcPort, (uint8_t*)response, responseLen);
    // ESP_LOGI(TAG, "Replied: %s", response);
}

void udp_task(void *arg)
{
    while (true) {
        socket_udp_recv(0);
        vTaskDelay(1);
    }
}

extern "C" void app_main(void)
{

    gpio_set_direction(CHIP_SELECT_SD, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(CHIP_SELECT_W5500_1, GPIO_MODE_OUTPUT);

    gpio_set_level(CHIP_SELECT_SD, true);
    gpio_set_level(CHIP_SELECT_W5500_0, true);
    gpio_set_level(CHIP_SELECT_W5500_1, true);

    spi_bus_config_t busCfg = {
        .mosi_io_num   = SPI_MOSI_1,
        .miso_io_num   = SPI_MISO_1,
        .sclk_io_num   = SPI_CLK_1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };
    spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devCfg = {
        .mode             = 0,
        .cs_ena_posttrans = 2,
        .clock_speed_hz   = 40 * 1000 * 1000,
        .spics_io_num     = CHIP_SELECT_W5500_1,
        .queue_size       = 1,
    };
    spi_bus_add_device(SPI2_HOST, &devCfg, &spiDevice);

    w5500_net_init();
    socket_udp_open(0, 5000);   // listen on UDP port 5000

    xTaskCreatePinnedToCore(
        udp_task,
        "udp",
        4096,
        NULL,
        10,
        NULL,
        1
    );

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