#include "motor.hpp"
#include <micro-motor/micro-motor.hpp>
#include <modm/debug/logger.hpp>
using StatusBits = modm_canopen::cia402::StatusBits;

static uint16_t adc_u_value = 0x7ff;
static uint16_t adc_v_value = 0x7ff;

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
	auto currents = micro_motor::getClarkePhaseCurrents(adc_u_value, adc_v_value);
	current_.updateCurrentAverage(std::get<0>(currents), std::get<1>(currents));
	// MODM_LOG_INFO << "O " << current_.getOrientedCurrent() << modm::endl;
	// MODM_LOG_INFO << "M " << current_.getMagnitude() << modm::endl;
	// MODM_LOG_INFO << "A " << current_.getAngleDifference() << modm::endl;
	// MODM_LOG_INFO << std::get<0>(currents) << " " << std::get<1>(currents) << modm::endl;

	auto curr_u = micro_motor::convertAdcToCurrent(adc_u_value);
	auto curr_v = micro_motor::convertAdcToCurrent(adc_v_value);
	auto curr_w = -curr_u - curr_v;
	auto curr_max = std::max(std::max(std::abs(curr_u), std::abs(curr_v)), std::abs(curr_w));
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
	using AdcU = Board::MotorCurrent::AdcU;
	using AdcV = Board::MotorCurrent::AdcV;

	::adc_u_value = AdcU::getValue();
	::adc_v_value = AdcV::getValue();
	AdcV::acknowledgeInterruptFlags(AdcV::InterruptFlag::EndOfRegularConversion |
									AdcV::InterruptFlag::EndOfSampling |
									AdcV::InterruptFlag::Overrun);
	AdcU::acknowledgeInterruptFlags(AdcU::InterruptFlag::EndOfRegularConversion |
									AdcU::InterruptFlag::EndOfSampling |
									AdcU::InterruptFlag::Overrun);
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
	Motor0.updateMotor();
}
}  // namespace micro_motor