
#pragma once

#include <atomic>
#include <cstdint>

#include "PID.hpp"
#include "SerialComm.hpp"
#include "Controller.hpp"

class GlobalVariableManager {
public:
    GlobalVariableManager();

    // Measurements / state
    int32_t getRotations();
    void setRotations(int32_t value);

    float getAngle();
    void setAngle(float value);

    float getCumAngle();
    void setCumAngle(float value);

    float getVelocity();
    void setVelocity(float value);

    float getAvgVelocity();
    void setAvgVelocity(float value);

    float getAcceleration();
    void setAcceleration(float value);

    float getAvgAcceleration();
    void setAvgAcceleration(float value);

    float getAvgStrength();
    void setAvgStrength(float value);

    float getAvgLoopTime();
    void setAvgLooptime(float value);

    uint32_t getVoltage();
    void setVoltage(uint32_t value);

    uint32_t getDrivingMode();
    void setDrivingMode(uint32_t value);

    float getAngleOffset();
    void setAngleOffset(float value);

    int32_t getCurrentLimit();
    void setCurrentLimit(int32_t value);

    uint32_t getUpdateFreqTransmition();
    void setUpdateFreqTransmition(uint32_t value);

    // Torque
    float getTorqueSetpoint();
    void setTorqueSetpoint(float value);

    uint32_t getUpdateFreqTorque();
    void setUpdateFreqTorque(uint32_t value);

    // Velocity
    float getVelocitySetpoint();
    void setVelocitySetpoint(float value);

    uint32_t getUpdateFreqVelocity();
    void setUpdateFreqVelocity(uint32_t value);

    // Position
    float getPositionSetpoint();
    void setPositionSetpoint(float value);

    uint32_t getUpdateFreqPosition();
    void setUpdateFreqPosition(uint32_t value);


private:
    void atomic_store_float(std::atomic_uint32_t& atomicValue, float value);
    float atomic_load_float(std::atomic_uint32_t& atomicValue);

    std::atomic_uint32_t _rotations{0};

    std::atomic_uint32_t _angle{0};
    std::atomic_uint32_t _cum_angle{0};
    std::atomic_uint32_t _velocity{0};
    std::atomic_uint32_t _acceleration{0};

    std::atomic_uint32_t _avg_velocity{0};
    std::atomic_uint32_t _avg_acceleration{0};
    std::atomic_uint32_t _avg_strength{0};
    std::atomic_uint32_t _avg_loop_time{0};

    float _angle_offset{0.0f};

    std::atomic_uint32_t _voltage{0};
    std::atomic_uint32_t _driving_mode{0};
    std::atomic_uint32_t _current_limit{0};

    std::atomic_uint32_t _update_freq_transmition{100};     // ms

    std::atomic_uint32_t _torque_setpoint{0};
    std::atomic_uint32_t _update_freq_torque{1};

    std::atomic_uint32_t _velocity_setpoint{0};
    std::atomic_uint32_t _update_freq_velocity{10};

    std::atomic_uint32_t _position_setpoint{0};
    std::atomic_uint32_t _update_freq_position{100};
};