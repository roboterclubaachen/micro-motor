#include "micro-motor.hpp"
#include <modm/debug/logger.hpp>
namespace micro_motor
{
static uint16_t adc_u_value = 0;
static uint16_t adc_v_value = 0;

uint16_t
getAdcUValue()
{
	return adc_u_value;
}
uint16_t
getAdcVValue()
{
	return adc_v_value;
}

MODM_ISR(TIM1_UP_TIM16)
{
	using AdcU = Board::MotorCurrent::AdcU;
	using AdcV = Board::MotorCurrent::AdcV;

	adc_u_value = AdcU::getValue();
	adc_v_value = AdcV::getValue();
	AdcV::acknowledgeInterruptFlags(AdcV::InterruptFlag::EndOfRegularConversion |
									AdcV::InterruptFlag::EndOfSampling |
									AdcV::InterruptFlag::Overrun);
	AdcU::acknowledgeInterruptFlags(AdcU::InterruptFlag::EndOfRegularConversion |
									AdcU::InterruptFlag::EndOfSampling |
									AdcU::InterruptFlag::Overrun);
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
}

}  // namespace micro_motor