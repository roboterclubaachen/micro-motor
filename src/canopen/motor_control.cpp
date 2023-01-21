#include "motor_control.hpp"
#include <modm/debug/logger.hpp>
#include <cmath>
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
	switch (mode_)
	{
		case OperatingMode::Voltage:
			outputPWM_ = commandedPWM_;
			targetReached_ = true;
			break;
		case OperatingMode::Velocity:
			velocityError_ = (commandedVelocity_ - actualVelocity_.getValue());
			velocityPid_.update(velocityError_);
			outputPWM_ = velocityPid_.getValue();
			targetReached_ = (velocityError_ == 0);
			break;
		case OperatingMode::Position:
			positionError_ = commandedPosition_ - actualPosition_;
			positionPid_.update(positionError_);
			velocityError_ = positionPid_.getValue() - actualVelocity_.getValue();
			velocityPid_.update(velocityError_);
			outputPWM_ = velocityPid_.getValue();

			if ((uint32_t)std::abs(positionError_) <= positionWindow_)
			{
				if (inPositionWindow_ < positionWindowTime_) inPositionWindow_++;
			} else
			{
				inPositionWindow_ = 0;
			}
			if (inPositionWindow_ >= positionWindowTime_)
			{
				targetReached_ = true;
				// TODO maybe disable motor here?
				// outputPWM_ = 0;
			} else
			{
				targetReached_ = false;
			}
			break;
		case OperatingMode::Disabled:
			targetReached_ = false;
			break;
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
	status_.setBit<StatusBits::TargetReached>(targetReached_);
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
