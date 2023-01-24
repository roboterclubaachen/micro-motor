#pragma once
#include <modm/math/filter/moving_average.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>
#include <modm-canopen/cia402/factors.hpp>

using OperatingMode = modm_canopen::cia402::OperatingMode;
using StateMachine = modm_canopen::cia402::StateMachine;
using ControlWord = modm_canopen::cia402::CommandWord;
using Factors = modm_canopen::cia402::Factors;

struct MotorState
{
	OperatingMode mode_{OperatingMode::Voltage};
	StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	ControlWord control_{0};
	Factors scalingFactors_{};

	int32_t actualPosition_{};
	int32_t lastPosition_{};

	modm::filter::MovingAverage<int32_t, 16> actualVelocity_{};

	bool enableMotor_{true};

	int16_t outputPWM_{};
};