#include <stdio.h>

#include "driver/i2c_master.h"
#include "I2CManager.hpp"

I2CManager::I2CManager(gpio_num_t sda, gpio_num_t scl) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };

    i2c_master_bus_handle_t bus;

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus)
    );

    _currentSensor = std::make_unique<INA226>(bus, 0x40, 100000);
    // Should probably not hardcode this.
    // Shunt = 3.5 mOhm | Max Current = 30A
    _currentSensor->InitDriver(3.5, 30.0);


    _multiplexer.begin(bus, 0x20, 100000);
    // Should probably not hardcode this.
    // 0 => Output | 1 => Input | x => Don't care
    //                         000001110xxxxxxx
    _multiplexer.setPortMode(0b0000011100000001);
}

bool I2CManager::readPin(int pin) {
    bool value;
    _multiplexer.readPin(pin, value);
    return value;
}

void I2CManager::writePin(int pin, bool value) {
    _multiplexer.writePin(pin, value);
}

int32_t I2CManager::getBusVoltage_mV() {
    return _currentSensor->GetBusVoltage_mV();
}

int32_t I2CManager::getCurrent_mA() {
    return _currentSensor->GetCurrent_uA() * 0.001;
}
