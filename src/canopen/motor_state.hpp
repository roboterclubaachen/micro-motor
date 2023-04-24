#pragma once
#include <modm/math/filter/moving_average.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>
#include <modm-canopen/cia402/factors.hpp>
#include <modm/architecture/interface/clock.hpp>

using OperatingMode = modm_canopen::cia402::OperatingMode;
using StateMachine = modm_canopen::cia402::StateMachine;
using ControlWord = modm_canopen::cia402::CommandWord;
using Factors = modm_canopen::cia402::Factors;

struct StateObjects
{
	static constexpr modm_canopen::Address ControlWord{0x6040, 0};
	static constexpr modm_canopen::Address StatusWord{0x6041, 0};
	static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
	static constexpr modm_canopen::Address ModeOfOperationDisplay{0x6061, 0};

	static constexpr modm_canopen::Address UpdateTime{0x2001, 0};  // Micro seconds

	static constexpr modm_canopen::Address PositionInternalValue{0x6063, 0};  // internal units
	static constexpr modm_canopen::Address PositionActualValue{0x6064, 0};    // User units

	static constexpr modm_canopen::Address VelocityActualValue{0x606C, 0};  // User units

	static constexpr modm_canopen::Address OutputPWM{0x2003, 0};  // Custom
	static constexpr modm_canopen::Address Reset{0x2007, 0};      // Set 1/0

	static constexpr modm_canopen::Address PositionFactorNumerator{0x6093, 1};
	static constexpr modm_canopen::Address PositionFactorDivisor{0x6093, 2};
	static constexpr modm_canopen::Address VelocityFactorNumerator{0x6094, 1};
	static constexpr modm_canopen::Address VelocityFactorDivisor{0x6094, 2};
	static constexpr modm_canopen::Address AccelerationFactorNumerator{0x6097, 1};
	static constexpr modm_canopen::Address AccelerationFactorDivisor{0x6097, 2};
	static constexpr modm_canopen::Address Polarity{0x607E, 0};
};

struct MotorState
{
	OperatingMode mode_{OperatingMode::Voltage};
	StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	ControlWord control_{0};
	Factors scalingFactors_{};

	modm::filter::MovingAverage<uint32_t, 32> updateTime_{};
	modm::chrono::micro_clock::time_point lastUpdate_{};

	int32_t actualPosition_{};
	int32_t lastPosition_{};

	modm::filter::MovingAverage<int32_t, 16> actualVelocity_{};

	bool enableMotor_{true};
	bool resetMotor_{false};

	int16_t outputPWM_{};
};