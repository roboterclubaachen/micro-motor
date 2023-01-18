#include "motor.hpp"
#include <modm/debug/logger.hpp>
using StatusBits = modm_canopen::cia402::StatusBits;

namespace
{
int_fast8_t
hallDiff(int_fast8_t oldState, int_fast8_t newState)
{
	auto diff = newState - oldState;
	if (diff < -2)
		diff += 6;
	else if (diff > 3)
		diff -= 6;
	return diff;
}
}  // namespace

void
Motor::updateVelocity()
{
	actualVelocity_ = actualPosition_ - lastPosition_;
	lastPosition_ = actualPosition_;
}

void
Motor::updatePosition()
{
#ifndef __unix__
	const auto hallState = readHall();
#else
	const auto hallState = dummy_.hall();
#endif
	actualPosition_ += hallDiff(lastHallState_, hallState);
	lastHallState_ = hallState;
}

bool
Motor::update()
{

	updatePosition();
	auto state = status_.state();

	if (mode_ == OperatingMode::Voltage) { outputVoltage_ = commandedVoltage_; }

	bool updated = false;
	if (controlTimer_.execute())
	{

		updateVelocity();
		if (mode_ == OperatingMode::Velocity)
		{
			 velocityError_ = (commandedVelocity_ - actualVelocity_);
			velocityPid_.update(velocityError_);
			outputVoltage_ = velocityPid_.getValue();

		} else if (mode_ == OperatingMode::Position)
		{
			// TODO Is this the way you do
			// this i have no idea

			const auto pos_error = commandedPosition_ - actualPosition_;
			positionPid_.update(pos_error);
			 velocityError_ = positionPid_.getValue() - actualVelocity_;
			velocityPid_.update(velocityError_);
			outputVoltage_ = velocityPid_.getValue();
		}

		updated = true;
	}

	if (state != modm_canopen::cia402::State::OperationEnabled ||
		mode_ == OperatingMode::Disabled || halted_)
	{
#ifndef __unix__
		motor_.disable();
#endif
		outputVoltage_ = 0;
	}
#ifndef __unix__
	motor_.setSetpoint(outputVoltage_);
	motor_.update();
#else
	dummy_.setInputVoltageInt(outputVoltage_);
	dummy_.update(1.0f);
#endif
	updateStatus();
	return updated;
}

void
Motor::updateStatus()
{
	bool targetReached = true;
	if (mode_ == OperatingMode::Velocity)
	{
		targetReached = (actualVelocity_ == commandedVelocity_);
		status_.setBit<StatusBits::SpeedZero>(actualVelocity_ == 0);
	} else if (mode_ == OperatingMode::Position)
	{
		auto pos_error = commandedPosition_ - actualPosition_;
		targetReached = ((pos_error > 0 && (uint32_t)pos_error < positionWindow_) ||
						 (pos_error <= 0 && (uint32_t)(-pos_error) < positionWindow_));
	}
	status_.setBit<StatusBits::TargetReached>(targetReached);
	// TODO check if the voltage is present if we apply it or if we can apply it
	status_.setBit<StatusBits::VoltagePresent>(outputVoltage_ != 0);
}

void
Motor::halt()
{
	halted_ = true;
}
void
Motor::unhalt()
{
	halted_ = false;
}
