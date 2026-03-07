#include "Multiplexer.hpp"

#define REG_IODIRA 0x00
#define REG_GPIOA  0x12

Multiplexer::Multiplexer(i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = speed,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus, &cfg, &dev)
    );
}

esp_err_t Multiplexer::writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buffer[3];

    buffer[0] = reg;
    buffer[1] = value & 0xFF;
    buffer[2] = value >> 8;

    return i2c_master_transmit(dev, buffer, sizeof(buffer), 100);
}

esp_err_t Multiplexer::readRegister(uint8_t reg, uint16_t &value)
{
    uint8_t buffer[2];

    esp_err_t err = i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        buffer,
        2,
        100
    );

    if (err == ESP_OK)
        value = buffer[0] | (buffer[1] << 8);

    return err;
}

esp_err_t Multiplexer::setPortMode(uint16_t mode)
{
    return writeRegister(REG_IODIRA, mode);
}

esp_err_t Multiplexer::writePort(uint16_t value)
{
    return writeRegister(REG_GPIOA, value);
}

esp_err_t Multiplexer::setPin(uint8_t pin, bool level)
{
    uint16_t port;

    readRegister(REG_GPIOA, port);

    if(level)
        port |= (1 << pin);
    else
        port &= ~(1 << pin);

    return writeRegister(REG_GPIOA, port);
}