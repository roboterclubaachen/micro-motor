#include "motor.hpp"
#include <micro-motor/micro-motor.hpp>
#include <modm/debug/logger.hpp>
using StatusBits = modm_canopen::cia402::StatusBits;

static uint16_t adc_u_value = 0x7ff;
static uint16_t adc_v_value = 0x7ff;
#ifdef MICRO_MOTOR_HAS_ADC_W
static uint16_t adc_w_value = 0x7ff;
#endif

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
	auto currents = micro_motor::getClarkePhaseCurrents(adc_u_value, adc_v_value);
	current_.updateCurrentAverage(std::get<0>(currents), std::get<1>(currents));
	auto curr_u = micro_motor::convertAdcToCurrent(adc_u_value);
	auto curr_v = micro_motor::convertAdcToCurrent(adc_v_value);
	auto curr_max = std::max(std::abs(curr_u), std::abs(curr_v));
#ifdef MICRO_MOTOR_HAS_ADC_W
	auto curr_w = micro_motor::convertAdcToCurrent(adc_w_value);
	curr_max = std::max(curr_max, std::abs(curr_w));
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

#ifdef MICRO_MOTOR_HAS_ADC_W
	using AdcW = Board::MotorCurrent::AdcW;
	::adc_w_value = AdcW::getValue();
	AdcW::acknowledgeInterruptFlags(AdcW::InterruptFlag::EndOfRegularConversion |
									AdcW::InterruptFlag::EndOfSampling |
									AdcW::InterruptFlag::Overrun);
#endif

	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Break);
	Motor0.updateMotor();
}
}  // namespace micro_motor