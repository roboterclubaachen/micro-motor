#pragma once
#include <cstdint>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

using modm_canopen::cia402::OperatingMode;

struct MotorState
{
	uint32_t updateTime = 0;
	bool targetReached = true;

	int16_t commandedPWM = 0;
	int16_t outputPWM = 0;

	int32_t velocityValue = 0;
	int32_t positionValue = 0;
	float currentValue = 0.0f;

	int32_t velErrorValue = 0;
	int32_t posErrorValue = 0;
	float currentErrorValue = 0.0f;

	OperatingMode currMode = OperatingMode::Current;
	OperatingMode receivedMode = OperatingMode::Disabled;

	modm_canopen::cia402::CommandWord control_{0};
	modm_canopen::cia402::StateMachine stateMachine_{modm_canopen::cia402::State::SwitchOnDisabled};

	float vPID_kP = 1.0f;
	float vPID_kI = 0.0f;
	float vPID_kD = 0.0f;

	int32_t targetSpeed = 0;
	int32_t velDemand = 0;

	float pPID_kP = 1.0f;
	float pPID_kI = 0.0f;
	float pPID_kD = 0.0f;

	int32_t targetPosition = 0;
	int32_t posDemand = 0;

	float cPID_kP = 1.0f;
	float cPID_kI = 0.0f;
	float cPID_kD = 0.0f;

	float targetCurrent = 0.0f;
	float commandedCurrent = 0.0f;
};

static MotorState state_{};