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

Motor::Motor(MotorInfo info)
	: commutationOffset_{info.hall_offset}, motor_{info.hall_offset}

{
	MotorState0::initialize(info);
}

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
	auto phase_current = micro_motor::getADCCurrents();
	auto currents = micro_motor::clarkeTransform(phase_current[0], phase_current[1]);
	current_.updateCurrentAverage(std::get<0>(currents), std::get<1>(currents));
	auto curr_max = std::max(std::abs(phase_current[0]), std::abs(phase_current[1]));
#ifdef MICRO_MOTOR_HAS_ADC_W
	curr_max = std::max(curr_max, std::abs(phase_current[2]));
#endif
	max_current_.update(curr_max);
}

void
Motor::updateMotor()
{
	motor_.update();
}

namespace micro_motor
{
MODM_ISR(TIM1_UP_TIM16)
{
	updateADC();
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Break);
	Motor0.updateMotor();
}
}  // namespace micro_motor