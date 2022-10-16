#include "motor.hpp"

namespace
{

int_fast8_t hallDiff(int_fast8_t oldState, int_fast8_t newState)
{
	auto diff = newState - oldState;
	if (diff < -2)
		diff += 6;
	else if (diff > 3)
		diff -= 6;
	return diff;
}

}

void Motor::updateVelocity()
{
	actualVelocity_ = actualPosition_ - lastPosition_;
	lastPosition_ = actualPosition_;
}

void Motor::updatePosition()
{
	const auto hallState = readHall();
	actualPosition_ += hallDiff(lastHallState_, hallState);
	lastHallState_ = hallState;
}

bool Motor::update()
{
	updatePosition();

	if (mode_ == ControlMode::Voltage) {
		outputVoltage_ = commandedVoltage_;
	}

	bool updated = false;
	if (controlTimer_.execute()) {
		updateVelocity();
		if (mode_ == ControlMode::Velocity) {
			const auto error = commandedVelocity_ - actualVelocity_;
			velocityPid_.update(error);
			outputVoltage_ = velocityPid_.getValue();
		}
		updated = true;
	}

	if (!enabled_ || mode_ == ControlMode::Disabled) {
		motor_.disable();
		outputVoltage_ = 0;
	}

	motor_.setSetpoint(outputVoltage_);
	motor_.update();

	return updated;
}
