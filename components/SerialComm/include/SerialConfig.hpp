#pragma once
#include <cstdint>

// --- Shared data structures (must match PC side exactly) ---
#pragma pack(push, 1)
struct SensorData {
    uint32_t iteration;
    uint32_t timestamp_ms;
    float position;
    float velocity;
    float torque;
    float current;
    uint32_t loopTimeMotor;
    uint32_t loopTimeSerial;
};

struct Command {
    uint8_t command_type;
    int32_t value0;
    float value1;
};
#pragma pack(pop)

enum DrivingMode {
    Disabled = 0,
    Torque = 1,
    Velocity = 2,
    Position = 3
};

enum CommandType {
    NoCommand = 0,
    TorqueKp = 1,
    TorqueKi = 2
};