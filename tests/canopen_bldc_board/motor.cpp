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

Motor::Motor(uint_fast8_t commutationOffset)
	: commutationOffset_{commutationOffset}, motor_{commutationOffset}

{}

void
Motor::updatePosition()
{
	const auto hallState = readHall();
	actualPosition_ += hallDiff(lastHallState_, hallState);
	lastHallState_ = hallState;
}

bool
Motor::update()
{
	bool updated = false;
	updatePosition();
	if (controlTimer_.execute())
	{
		MotorControl0.setActualPosition(actualPosition_);
		MotorControl0.update();
		if (!MotorControl0.shouldEnableMotor()) { motor_.disable(); }
		motor_.setSetpoint(MotorControl0.outputPWM());
		updated = true;
	}
	motor_.update();
	return updated;
}
