#pragma once
#include <modm-canopen/canopen_device.hpp>
#include "motor_control.hpp"
#include "canopen_objects.hpp"

struct FactorHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;
		map.template setReadHandler<Objects::PositionFactorNumerator>(
			+[]() { return MotorControl0.scalingFactors().position.numerator; });

		map.template setWriteHandler<Objects::PositionFactorNumerator>(+[](uint32_t value) {
			MotorControl0.scalingFactors().position.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionFactorDivisor>(
			+[]() { return MotorControl0.scalingFactors().position.divisor; });

		map.template setWriteHandler<Objects::PositionFactorDivisor>(+[](uint32_t value) {
			MotorControl0.scalingFactors().position.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::VelocityFactorNumerator>(
			+[]() { return MotorControl0.scalingFactors().velocity.numerator; });

		map.template setWriteHandler<Objects::VelocityFactorNumerator>(+[](uint32_t value) {
			MotorControl0.scalingFactors().velocity.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::VelocityFactorDivisor>(
			+[]() { return MotorControl0.scalingFactors().velocity.divisor; });

		map.template setWriteHandler<Objects::VelocityFactorDivisor>(+[](uint32_t value) {
			MotorControl0.scalingFactors().velocity.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::AccelerationFactorNumerator>(
			+[]() { return MotorControl0.scalingFactors().acceleration.numerator; });

		map.template setWriteHandler<Objects::AccelerationFactorNumerator>(+[](uint32_t value) {
			MotorControl0.scalingFactors().acceleration.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::AccelerationFactorDivisor>(
			+[]() { return MotorControl0.scalingFactors().acceleration.divisor; });

		map.template setWriteHandler<Objects::AccelerationFactorDivisor>(+[](uint32_t value) {
			MotorControl0.scalingFactors().acceleration.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::Polarity>(
			+[]() { return MotorControl0.scalingFactors().getPolarity(); });

		map.template setWriteHandler<Objects::Polarity>(+[](uint8_t value) {
			MotorControl0.scalingFactors().setPolarity(value);
			return SdoErrorCode::NoError;
		});
	}
};
