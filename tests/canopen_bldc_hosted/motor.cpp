#include "motor.hpp"
#include <modm/debug/logger.hpp>
#include <micro-motor/canopen/motor_control.hpp>
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

Motor::Motor(uint_fast8_t commutationOffset) : commutationOffset_{commutationOffset} {}

void
Motor::updatePosition()
{
	const auto hallState = dummy_.hall();
	actualPosition_ += hallDiff(lastHallState_, hallState);
	lastHallState_ = hallState;
}

bool
Motor::update()
{

	updatePosition();
	MotorControl0.setActualPosition(actualPosition_);
	bool updated = MotorControl0.update();
	dummy_.setInputVoltageInt(MotorControl0.outputPWM());
	dummy_.update(0.1f);
	return updated;
}