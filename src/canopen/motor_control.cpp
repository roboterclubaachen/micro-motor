#include "motor_control.hpp"
#include <modm/debug/logger.hpp>
using StatusBits = modm_canopen::cia402::StatusBits;

MotorControl::MotorControl(const Pid::Parameter &velocityParameters,
						   const Pid::Parameter &positionParameters)
{
	setVelocityPidParams(velocityParameters);
	setPositionPidParams(positionParameters);
}

void
MotorControl::updateVelocity()
{
	actualVelocity_.update(actualPosition_ - lastPosition_);
	lastPosition_ = actualPosition_;
}

void
MotorControl::update()
{
	const auto state = status_.state();
	updateVelocity();

	if (mode_ == OperatingMode::Voltage) { outputPWM_ = commandedPWM_; }

	if (mode_ == OperatingMode::Velocity)
	{
		velocityError_ = (commandedVelocity_ - actualVelocity_.getValue());
		velocityPid_.update(velocityError_);
		outputPWM_ = velocityPid_.getValue();

	} else if (mode_ == OperatingMode::Position)
	{
		// TODO Is this the way you do
		// this i have no idea

		positionError_ = commandedPosition_ - actualPosition_;
		positionPid_.update(positionError_);
		velocityError_ = positionPid_.getValue() - actualVelocity_.getValue();
		velocityPid_.update(velocityError_);
		outputPWM_ = velocityPid_.getValue();
	}

	if (state != modm_canopen::cia402::State::OperationEnabled ||
		mode_ == OperatingMode::Disabled || halted_)
	{
		enableMotor_ = false;
		outputPWM_ = 0;
	} else
	{
		enableMotor_ = true;
	}
	updateStatus();
}

void
MotorControl::updateStatus()
{
	bool targetReached = true;
	if (mode_ == OperatingMode::Velocity)
	{
		targetReached = (actualVelocity_.getValue() == commandedVelocity_);
		status_.setBit<StatusBits::SpeedZero>(actualVelocity_.getValue() == 0);
	} else if (mode_ == OperatingMode::Position)
	{
		auto pos_error = commandedPosition_ - actualPosition_;
		targetReached = ((pos_error > 0 && (uint32_t)pos_error < positionWindow_) ||
						 (pos_error <= 0 && (uint32_t)(-pos_error) < positionWindow_));
	}
	status_.setBit<StatusBits::TargetReached>(targetReached);
	// TODO check if the voltage is present if we apply it or if we can apply it
	status_.setBit<StatusBits::VoltagePresent>(outputPWM_ != 0);
}

void
MotorControl::halt()
{
	halted_ = true;
}
void
MotorControl::unhalt()
{
	halted_ = false;
}
