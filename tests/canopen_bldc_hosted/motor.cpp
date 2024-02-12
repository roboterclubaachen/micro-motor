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

modm::PreciseClock::time_point
Motor::lastUpdateTime() const
{
	return lastUpdate_;
}