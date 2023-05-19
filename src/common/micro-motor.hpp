#pragma once
#include "hardware.hpp"
#include <tuple>
#include <cmath>
#include <modm/debug/logger.hpp>
namespace micro_motor
{
uint16_t
getAdcUValue();
uint16_t
getAdcVValue();

inline void
initialize()
{
	using MotorTimer = Board::Motor::MotorTimer;
	MotorTimer::enableInterruptVector(MotorTimer::Interrupt::Update, true, 5);
	MotorTimer::enableInterrupt(MotorTimer::Interrupt::Update);
}

constexpr std::tuple<float, float>
clarkeTransform(float u, float v)
{
	constexpr float one_by_sqrt3 = (float)(1.0 / std::sqrt(3.0));
	constexpr float two_by_sqrt3 = (float)(2.0 / std::sqrt(3.0));

	return {u, u * one_by_sqrt3 + v * two_by_sqrt3};
}

constexpr float
convertAdcToCurrent(uint16_t adcValue)
{
	// return adcValue - 0x7ff;

	constexpr float ShuntResistance = 5e-3;
	constexpr float CurrentGain = 50;
	constexpr float ReferenceVoltage = 2.9;
	constexpr uint16_t AdcCounts = (1 << 12) - 1;

	const float adcVoltage = adcValue * (ReferenceVoltage / AdcCounts);
	const float current =
		(adcVoltage - (ReferenceVoltage / 2)) * (1 / (CurrentGain * ShuntResistance));
	return current;
}

constexpr uint16_t
convertCurrentToAdc(float current)
{
	constexpr float ShuntResistance = 5e-3;
	constexpr float CurrentGain = 50;
	constexpr float ReferenceVoltage = 2.9;
	constexpr uint16_t AdcCounts = (1 << 12) - 1;
	const float adcVoltage =
		(current / (1 / CurrentGain * ShuntResistance)) + (ReferenceVoltage / 2);
	const float adcValue = adcVoltage / (ReferenceVoltage / AdcCounts);
	return (uint16_t)adcValue;
}

inline void
setCurrentLimitAmps(float limit)
{
	uint16_t limit_12_bit = convertCurrentToAdc(limit) & 0x0FFF;
	DAC3->DHR12R1 = limit_12_bit;
	DAC3->DHR12R2 = limit_12_bit;
}

inline std::tuple<float, float>
getClarkePhaseCurrents()
{
	return clarkeTransform(convertAdcToCurrent(getAdcUValue()),
						   convertAdcToCurrent(getAdcVValue()));
}

};  // namespace micro_motor