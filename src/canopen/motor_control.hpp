#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/moving_average.hpp>
#include <modm/math/filter/s_curve_controller.hpp>
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

class MotorControl
{
private:
	modm::PeriodicTimer controlTimer_{10ms};

	OperatingMode mode_{OperatingMode::Voltage};
	StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	ControlWord control_{0};
	Factors scalingFactors_{};

	// Safety
	bool halted_{false};

	// Position Mode
	Pid::Parameter positionPidParameters_;
	Pid positionPid_;
	bool receivedPositionRelative_{true};
	int32_t receivedPosition_{};
	int32_t commandedPosition_{};
	uint32_t positionWindow_{};
	int32_t positionError_{};

	// Velocity Mode
	Pid::Parameter velocityPidParameters_;
	Pid velocityPid_;
	int32_t receivedVelocity_{};
	int32_t commandedVelocity_{};
	int32_t velocityError_{};

	// Voltage Mode
	int16_t commandedPWM_{};

	// General State
	int32_t actualPosition_{};
	int32_t lastPosition_{};
	modm::filter::MovingAverage<int32_t, 16> actualVelocity_{};
	int16_t outputPWM_{};
	bool enableMotor_{true};

	void
	updateVelocity();
	void
	updateStatus();

public:
	MotorControl(const Pid::Parameter &velocityParameters,
				 const Pid::Parameter &positionParameters);

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
	commandedPWM() const
	{
		return commandedPWM_;
	}
	void
	setCommandedPWM(int16_t pwm)
	{
		commandedPWM_ = pwm;
	}

	int32_t
	commandedPosition() const
	{
		return commandedPosition_;
	}
		int32_t
	positionError() const
	{
		return positionError_;
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
		return actualVelocity_.getValue();
	}
	int32_t
	position() const
	{
		return actualPosition_;
	}
	int16_t
	outputPWM() const
	{
		return outputPWM_;
	}
	bool
	shouldEnableMotor() const
	{
		return enableMotor_;
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

	inline void
	setActualPosition(int32_t position)
	{
		actualPosition_ = position;
	}

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

	inline Pid::Parameter
	getVelocityPidParams()
	{
		return velocityPidParameters_;
	}

	inline void
	setVelocityPidParams(const Pid::Parameter &params)
	{
		velocityPidParameters_ = params;
		velocityPid_.setParameter(velocityPidParameters_);
	}

	inline Pid::Parameter
	getPositionPidParams()
	{
		return positionPidParameters_;
	}

	inline void
	setPositionPidParams(const Pid::Parameter &params)
	{
		positionPidParameters_ = params;
		positionPid_.setParameter(positionPidParameters_);
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

inline MotorControl MotorControl0{velocityControllerParameters, positionControllerParameters};
