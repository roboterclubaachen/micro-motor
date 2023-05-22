#include "motor.hpp"
#include <micro-motor/micro-motor.hpp>
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

void
Motor::updateCurrent()
{
	auto currents = micro_motor::getClarkePhaseCurrents();
	current_.updateCurrentAverage(std::get<0>(currents), std::get<1>(currents));
	// MODM_LOG_INFO << "O " << current_.getOrientedCurrent() << modm::endl;
	// MODM_LOG_INFO << "M " << current_.getMagnitude() << modm::endl;
	// MODM_LOG_INFO << "A " << current_.getAngleDifference() << modm::endl;
	// MODM_LOG_INFO << std::get<0>(currents) << " " << std::get<1>(currents) << modm::endl;
}
