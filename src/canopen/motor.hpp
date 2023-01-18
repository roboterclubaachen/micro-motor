#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#ifndef __unix__
#include <librobots2/motor/bldc_motor_block_commutation.hpp>
#include <micro-motor/hardware.hpp>
#else
#include <micro-motor/test/sim_motor.hpp>
#endif
#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer.hpp>

#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>
#include <modm-canopen/cia402/factors.hpp>

using namespace std::literals;

using OperatingMode = modm_canopen::cia402::OperatingMode;
using StateMachine = modm_canopen::cia402::StateMachine;
using ControlWord = modm_canopen::cia402::CommandWord;
using Factors = modm_canopen::cia402::Factors;
using Pid = modm::Pid<float>;

class Motor
{
private:
	OperatingMode mode_{OperatingMode::Voltage};
	StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	ControlWord control_{0};
	Factors scalingFactors_{};

	int32_t receivedVelocity_{};
	bool receivedPositionRelative_{true};
	int32_t receivedPosition_{};
	int32_t commandedVelocity_{};
	int32_t commandedPosition_{};
	uint32_t positionWindow_{};
	int32_t actualVelocity_{};
	int32_t actualPosition_{};
	int16_t commandedVoltage_{};
	int16_t outputVoltage_{};
	int32_t velocityError_{};
	bool halted_{false};

	modm::PeriodicTimer controlTimer_{10ms};
	Pid velocityPid_;
	Pid positionPid_;
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t lastPosition_{};

#ifndef __unix__
	librobots2::motor::BldcMotorBlockCommutation<Board::Motor> motor_;
	using Hall = librobots2::motor::HallPermutations<Board::Motor::HallPort>;
	uint_fast8_t
	readHall()
	{
		const auto &HallStates = librobots2::motor::block_commutation::SequenceLut;
		return HallStates[Hall::read(commutationOffset_) & 0b111];
	}
#else
	MotorSimulation dummy_{};
#endif

	void
	updateVelocity();
	void
	updatePosition();
	void
	updateStatus();

public:
	Motor(uint_fast8_t commutationOffset, const Pid::Parameter &velocityParameters,
		  const Pid::Parameter &positionParameters)
		: commutationOffset_{commutationOffset}
#ifndef __unix__
		  ,
		  motor_{commutationOffset}
#endif
	{
		velocityPid_.setParameter(velocityParameters);
		positionPid_.setParameter(positionParameters);
	}
	void
	initializeHall()
	{
#ifndef __unix__
		lastHallState_ = readHall();
#else
		lastHallState_ = dummy_.hall();
#endif
	}

#ifdef __unix__
	inline MotorSimulation &
	dummy()
	{
		return dummy_;
	};
#endif

	OperatingMode
	mode()
	{
		return mode_;
	}
	void
	setMode(OperatingMode mode)
	{
		mode_ = mode;
	}
	int32_t
	velocityError() const
	{
		return velocityError_;
	}
	int32_t
	commandedVelocity() const
	{
		return commandedVelocity_;
	}
	void
	setCommandedVelocity(int32_t velocity)
	{
		commandedVelocity_ = velocity;
	}

	int16_t
	commandedVoltage() const
	{
		return commandedVoltage_;
	}
	void
	setCommandedVoltage(int16_t voltage)
	{
		commandedVoltage_ = voltage;
	}

	int32_t
	commandedPosition() const
	{
		return commandedPosition_;
	}
	int32_t
	receivedPosition() const
	{
		return receivedPosition_;
	}
	void
	setReceivedPosition(int32_t position)
	{
		receivedPosition_ = position;
	}

	uint32_t
	positionWindow() const
	{
		return positionWindow_;
	}
	void
	setPositionWindow(uint32_t window)
	{
		positionWindow_ = window;
	}

	int32_t
	velocity() const
	{
		return actualVelocity_;
	}
	int32_t
	position() const
	{
		return actualPosition_;
	}
	int16_t
	outputVoltage() const
	{
		return outputVoltage_;
	}

	bool
	updatePositionSetPoint(bool relative)
	{
		if (mode_ != OperatingMode::Position) return false;
		if (relative)
		{
			commandedPosition_ += receivedPosition_;
		} else
		{
			commandedPosition_ = receivedPosition_;
		}
		return true;
	}

	void
	halt();
	void
	unhalt();
	bool
	update();

	inline ControlWord &
	control()
	{
		return control_;
	}

	inline StateMachine &
	status()
	{
		return status_;
	}

	inline Factors &
	scalingFactors()
	{
		return scalingFactors_;
	}
};

const Pid::Parameter velocityControllerParameters{
	// TODO
	1.0f,                                       // kp
	0.0f,                                       // ki
	0.0f,                                       // kd
	100.0f,                                     // max error sum
	(float)std::numeric_limits<int16_t>::max()  // max output
};

const Pid::Parameter positionControllerParameters{
	// TODO
	1.0f,                                       // kp
	0.0f,                                       // ki
	0.0f,                                       // kd
	0.0f,                                       // max error sum
	(float)std::numeric_limits<int32_t>::max()  // max output
};

constexpr uint8_t motor0CommutationOffset{1};
inline Motor Motor0{motor0CommutationOffset, velocityControllerParameters,
					positionControllerParameters};
