#include "micro-motor.hpp"
#include <cstdint>

namespace micro_motor
{
static uint16_t adc_u_value = 0x7ff;
static uint16_t adc_v_value = 0x7ff;
#ifdef MICRO_MOTOR_HAS_ADC_W
static uint16_t adc_w_value = 0x7ff;
#endif

std::array<uint16_t, 3>
getADCValues()
{
	#ifdef MICRO_MOTOR_HAS_ADC_W
	return {adc_u_value, adc_v_value, adc_w_value };
	#else
	int adc_w_value = -((int)adc_u_value - 0x7ff) - ((int)adc_v_value - 0x7ff);

	return {adc_u_value, adc_v_value, (uint16_t)(adc_w_value + 0x7ff)};
	#endif
}

std::array<float, 3>
getADCCurrents()
{
	float u = convertAdcToCurrent(adc_u_value);
	float v = convertAdcToCurrent(adc_v_value);
	#ifdef MICRO_MOTOR_HAS_ADC_W
	float w = convertAdcToCurrent(adc_w_value);
	#else
	float w = -u - v;
	#endif
	return {u, v, w};
}

void
updateADC()
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

#ifdef MICRO_MOTOR_HAS_ADC_W
	using AdcW = Board::MotorCurrent::AdcW;
	adc_w_value = AdcW::getValue();
	AdcW::acknowledgeInterruptFlags(AdcW::InterruptFlag::EndOfRegularConversion |
									AdcW::InterruptFlag::EndOfSampling |
									AdcW::InterruptFlag::Overrun);
#endif
}

}  // namespace micro_motor