#include "GlobalVariableManager.hpp"

#include <cstring>


GlobalVariableManager::GlobalVariableManager() = default;

void GlobalVariableManager::atomic_store_float(std::atomic_uint32_t& atomicValue, float value) {
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    atomicValue.store(bits, std::memory_order_relaxed);
}

float GlobalVariableManager::atomic_load_float(std::atomic_uint32_t& atomicValue) {
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
    uint32_t bits = atomicValue.load(std::memory_order_relaxed);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// Measurements / states
int32_t GlobalVariableManager::getRotations() {
    return static_cast<int32_t>(_rotations.load(std::memory_order_relaxed));
}

void GlobalVariableManager::setRotations(int32_t value) {
    _rotations.store(static_cast<uint32_t>(value), std::memory_order_relaxed);
}

float GlobalVariableManager::getAngle() {
    return atomic_load_float(_angle);
}

void GlobalVariableManager::setAngle(float value) {
    atomic_store_float(_angle, value);
}

float GlobalVariableManager::getCumAngle() {
    return atomic_load_float(_cum_angle);
}

void GlobalVariableManager::setCumAngle(float value) {
    atomic_store_float(_cum_angle, value);
}

float GlobalVariableManager::getVelocity() {
    return atomic_load_float(_velocity);
}

void GlobalVariableManager::setVelocity(float value) {
    atomic_store_float(_velocity, value);
}

float GlobalVariableManager::getAvgVelocity() {
    return atomic_load_float(_avg_velocity);
}

void GlobalVariableManager::setAvgVelocity(float value) {
    atomic_store_float(_avg_velocity, value);
}

float GlobalVariableManager::getAcceleration() {
    return atomic_load_float(_acceleration);
}

void GlobalVariableManager::setAcceleration(float value) {
    atomic_store_float(_acceleration, value);
}

float GlobalVariableManager::getAvgAcceleration() {
    return atomic_load_float(_avg_acceleration);
}

void GlobalVariableManager::setAvgAcceleration(float value) {
    atomic_store_float(_avg_acceleration, value);
}

float GlobalVariableManager::getAvgStrength() {
    return atomic_load_float(_avg_strength);
}

void GlobalVariableManager::setAvgStrength(float value) {
    atomic_store_float(_avg_strength, value);
}

float GlobalVariableManager::getAvgLoopTime() {
    return atomic_load_float(_avg_loop_time);
}

void GlobalVariableManager::setAvgLooptime(float value) {
    atomic_store_float(_avg_loop_time, value);
}

uint32_t GlobalVariableManager::getVoltage() {
    return _voltage.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setVoltage(uint32_t value) {
    _voltage.store(value, std::memory_order_relaxed)
}

uint32_t GlobalVariableManager::getDrivingMode() {
    return _driving_mode.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setDrivingMode(uint32_t value) {
    _driving_mode.store(value, std::memory_order_relaxed);
}

float GlobalVariableManager::getAngleOffset() {
    return _angle_offset;
}

void GlobalVariableManager::setAngleOffset(float value) {
    _angle_offset = value;
}

int32_t GlobalVariableManager::getCurrentLimit() {
    return static_cast<int32_t>(_current_limit.load(std::memory_order_relaxed));
}

void GlobalVariableManager::setCurrentLimit(int32_t value) {
    _current_limit.store(static_cast<uint32_t>(value), std::memory_order_relaxed);
}

// Acceleration
float GlobalVariableManager::getAccelerationSetpoint() {
    return atomic_load_float(_acceleration_setpoint);
}

void GlobalVariableManager::setAccelerationSetpoint(float value) {
    atomic_store_float(_acceleration_setpoint, value);
}

uint32_t GlobalVariableManager::getUpdateFreqAcceleration() {
    return _update_freq_acceleration.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setUpdateFreqAcceleration(uint32_t value) {
    _update_freq_acceleration.store(value, std::memory_order_relaxed);
}

// Torque
float GlobalVariableManager::getTorqueSetpoint() {
    return atomic_load_float(_torque_setpoint);
}

void GlobalVariableManager::setTorqueSetpoint(float value) {
    atomic_store_float(_torque_setpoint, value);
}

uint32_t GlobalVariableManager::getUpdateFreqTorque() {
    return _update_freq_torque.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setUpdateFreqTorque(uint32_t value) {
    _update_freq_torque.store(value, std::memory_order_relaxed);
}

// Velocity
float GlobalVariableManager::getVelocitySetpoint() {
    return atomic_load_float(_velocity_setpoint);
}

void GlobalVariableManager::setVelocitySetpoint(float value) {
    atomic_store_float(_velocity_setpoint, value);
}

uint32_t GlobalVariableManager::getUpdateFreqVelocity() {
    return _update_freq_velocity.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setUpdateFreqVelocity(uint32_t value) {
    _update_freq_velocity.store(value, std::memory_order_relaxed);
}

// Position
float GlobalVariableManager::getPositionSetpoint() {
    return atomic_load_float(_position_setpoint);
}

void GlobalVariableManager::setPositionSetpoint(float value) {
    atomic_store_float(_position_setpoint, value);
}

uint32_t GlobalVariableManager::getUpdateFreqPosition() {
    return _update_freq_position.load(std::memory_order_relaxed);
}

void GlobalVariableManager::setUpdateFreqPosition(uint32_t value) {
    _update_freq_position.store(value, std::memory_order_relaxed);
}

// Commands FEIL Å HA I GLOB VAR? FINN UT
void GlobalVariableManager::handleCommand(Command& cmd) {
    switch (cmd.command_type)
    {
    case 1:
        setTorqueSetpoint(cmd.value1);
        break;
    
    default:
        break;
    }
}