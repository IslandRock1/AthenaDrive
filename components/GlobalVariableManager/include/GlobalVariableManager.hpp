
#pragma once

#include <atomic>
#include <cstdint>


class GlobalVariableManager {
public:
    GlobalVariableManager();

    // Measurements / state
    bool getWantedCalibrationMode();
    void setWantedCalibrationMode(bool value);

    bool getActualCalibrationMode();
    void setActualCalibrationMode(bool value);

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

    int32_t getCurrentLimit();
    void setCurrentLimit(int32_t value);

    uint32_t getUpdateFreqTransmition();
    void setUpdateFreqTransmition(uint32_t value);

    // Torque
    float getTorqueKp();
    void setTorqueKp(float value);

    float getTorqueKi();
    void setTorqueKi(float value);

    float getTorqueSetpoint();
    void setTorqueSetpoint(float value);

    uint32_t getUpdateFreqTorque();
    void setUpdateFreqTorque(uint32_t value);

    // Velocity
    float getVelocityKp();
    void setVelocityKp(float value);

    float getVelocityKi();
    void setVelocityKi(float value);

    float getVelocityKd();
    void setVelocityKd(float value);

    float getVelocitySetpoint();
    void setVelocitySetpoint(float value);

    uint32_t getUpdateFreqVelocity();
    void setUpdateFreqVelocity(uint32_t value);

    // Position
    float getPositionKp();
    void setPositionKp(float value);

    float getPositionKi();
    void setPositionKi(float value);

    float getPositionKd();
    void setPositionKd(float value);

    float getPositionSetpoint();
    void setPositionSetpoint(float value);

    uint32_t getUpdateFreqPosition();
    void setUpdateFreqPosition(uint32_t value);


private:
    void atomic_store_float(std::atomic_uint32_t& atomicValue, float value);
    float atomic_load_float(std::atomic_uint32_t& atomicValue);

    std::atomic_bool _wantedCalibrationMode{false};
    std::atomic_bool _actualCalibrationMode{false};
    std::atomic_uint32_t _rotations{0};

    std::atomic_uint32_t _angle{0};
    std::atomic_uint32_t _cum_angle{0};
    std::atomic_uint32_t _velocity{0};
    std::atomic_uint32_t _acceleration{0};

    std::atomic_uint32_t _avg_velocity{0};
    std::atomic_uint32_t _avg_acceleration{0};
    std::atomic_uint32_t _avg_strength{0};
    std::atomic_uint32_t _avg_loop_time{0};

    std::atomic_uint32_t _voltage{0};
    std::atomic_uint32_t _driving_mode{0};
    std::atomic_uint32_t _current_limit{0};

    std::atomic_uint32_t _update_freq_transmition{100};     // ms

    std::atomic_uint32_t _torqueKp;
    std::atomic_uint32_t _torqueKi;
    std::atomic_uint32_t _torque_setpoint{0};
    std::atomic_uint32_t _update_freq_torque{1};

    std::atomic_uint32_t _velocityKp;
    std::atomic_uint32_t _velocityKi;
    std::atomic_uint32_t _velocityKd;
    std::atomic_uint32_t _velocity_setpoint{0};
    std::atomic_uint32_t _update_freq_velocity{10};

    std::atomic_uint32_t _positionKp;
    std::atomic_uint32_t _positionKi;
    std::atomic_uint32_t _positionKd;
    std::atomic_uint32_t _position_setpoint{0};
    std::atomic_uint32_t _update_freq_position{100};
};