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
	float voltage;
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
	TorqueSetpoint = 1,
	TorqueKp = 2,
	TorqueKi = 3,
	TorqueKd = 4, // Not in use.
	VelocitySetpoint = 5,
	VelocityKp = 6,
	VelocityKi = 7,
	VelocityKd = 8,
	PositionSetpoint = 9,
	PositionKp = 10,
	PositionKi = 11,
	PositionKd = 12,
	DrivingModeCommand = 13,
	SetCurrentLimit = 14
};