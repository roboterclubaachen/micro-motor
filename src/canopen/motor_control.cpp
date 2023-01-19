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
	actualVelocity_.addValue(actualPosition_ - lastPosition_);
	lastPosition_ = actualPosition_;
}

bool
MotorControl::update()
{
	bool updated = false;
	auto state = status_.state();

	if (mode_ == OperatingMode::Voltage) { outputPWM_ = commandedPWM_; }

	if (controlTimer_.execute())
	{

		updateVelocity();
		if (mode_ == OperatingMode::Velocity)
		{
			velocityError_ = (commandedVelocity_ - actualVelocity_.average());
			velocityPid_.update(velocityError_);
			outputPWM_ = velocityPid_.getValue();

		} else if (mode_ == OperatingMode::Position)
		{
			// TODO Is this the way you do
			// this i have no idea

			const auto pos_error = commandedPosition_ - actualPosition_;
			positionPid_.update(pos_error);
			velocityError_ = positionPid_.getValue() - actualVelocity_.average();
			velocityPid_.update(velocityError_);
			outputPWM_ = velocityPid_.getValue();
		}

		updated = true;
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
	return updated;
}

void
MotorControl::updateStatus()
{
	bool targetReached = true;
	if (mode_ == OperatingMode::Velocity)
	{
		targetReached = (actualVelocity_.average() == commandedVelocity_);
		status_.setBit<StatusBits::SpeedZero>(actualVelocity_.average() == 0);
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
