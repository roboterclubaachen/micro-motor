#pragma once
#include <cstdint>

#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

using modm_canopen::cia402::OperatingMode;

// PIDs
constexpr float pPID_kP = 5.0f;
constexpr float pPID_kI = 0.01f;
constexpr float pPID_kD = 0.0f;


//constexpr float vPID_kP = 0.0002f;
//constexpr float vPID_kI = 0.000006f;
//constexpr float vPID_kD = 0.01f;

constexpr float vPID_kP = 0.0007f;
constexpr float vPID_kI = 0.00003f;
constexpr float vPID_kD = 0.0f;

constexpr bool invert = false;

struct State
{
	bool targetReached = true;
	uint32_t updateTime = 0;

	int32_t targetSpeed = 0;
	int32_t velocityValue = 0;
	int32_t velDemand = 0;
	int32_t velErrorValue = 0;

	int32_t targetPosition = 0;
	int32_t positionValue = 0;
	int32_t posDemand = 0;
	int32_t posErrorValue = 0;

	float targetCurrent = 0.0f;
	float currentValue = 0.0f;
	float commandedCurrent = 0.0f;
	float currentErrorValue = 0.0f;

	float maxCharge = 0.0f;
	float currentCharge = 0.0f;

	float orientedCurrent = 0.0f;
	float orientedCurrentAngleDiff = 0.0f;

	int16_t commandedPWM = 4000;
	int16_t outputPWM = 0;

	OperatingMode currMode = OperatingMode::Current;
	OperatingMode receivedMode = OperatingMode::Disabled;

	modm_canopen::cia402::CommandWord control_{0};
	modm_canopen::cia402::StateMachine state_{modm_canopen::cia402::State::SwitchOnDisabled};
};

static inline State state{};