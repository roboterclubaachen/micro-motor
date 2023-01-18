#pragma once

#include <modm-canopen/canopen_device.hpp>
#include <modm/debug/logger.hpp>

#include "motor.hpp"

struct Objects
{
	static constexpr modm_canopen::Address ControlWord{0x6040, 0};
	static constexpr modm_canopen::Address StatusWord{0x6041, 0};
	static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
	static constexpr modm_canopen::Address ModeOfOperationDisplay{0x6061, 0};

	static constexpr modm_canopen::Address PositionDemandValue{0x6062, 0};  // User units
	static constexpr modm_canopen::Address PositionActualValue{0x6064, 0};  // User units
	static constexpr modm_canopen::Address TargetPosition{0x607A, 0};       // User units
	static constexpr modm_canopen::Address PositionWindow{0x6067, 0};       // User units

	static constexpr modm_canopen::Address VoltageCommand{0x2002, 0};  // Custom
	static constexpr modm_canopen::Address OutputVoltage{0x2003, 0};   // Custom
	static constexpr modm_canopen::Address VelocityError{0x2004, 0};   // Custom

	static constexpr modm_canopen::Address VelocityActualValue{0x606C, 0};  // User units
	static constexpr modm_canopen::Address TargetVelocity{0x60FF, 0};       // User units

	static constexpr modm_canopen::Address PositionFactorNumerator{0x6093, 1};
	static constexpr modm_canopen::Address PositionFactorDivisor{0x6093, 2};
	static constexpr modm_canopen::Address VelocityFactorNumerator{0x6094, 1};
	static constexpr modm_canopen::Address VelocityFactorDivisor{0x6094, 2};
	static constexpr modm_canopen::Address AccelerationFactorNumerator{0x6097, 1};
	static constexpr modm_canopen::Address AccelerationFactorDivisor{0x6097, 2};
	static constexpr modm_canopen::Address Polarity{0x607E, 0};
};

struct CanOpenHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;

		map.template setReadHandler<Objects::ModeOfOperation>(
			+[]() { return int8_t(Motor0.mode()); });

		map.template setReadHandler<Objects::ModeOfOperationDisplay>(
			+[]() { return int8_t(Motor0.mode()); });

		map.template setWriteHandler<Objects::ModeOfOperation>(+[](int8_t value) {
			const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
							   (value == int8_t(OperatingMode::Voltage)) ||
							   (value == int8_t(OperatingMode::Velocity)) ||
							   (value == int8_t(OperatingMode::Position));

			if (valid)
			{
				Motor0.setMode(static_cast<OperatingMode>(value));
				return SdoErrorCode::NoError;
			} else
			{
				return SdoErrorCode::InvalidValue;
			}
		});

		map.template setReadHandler<Objects::PositionActualValue>(
			+[]() { return Motor0.scalingFactors().position.toUser(Motor0.position()); });

		map.template setReadHandler<Objects::VelocityActualValue>(
			+[]() { return Motor0.scalingFactors().velocity.toUser(Motor0.velocity()); });

		map.template setReadHandler<Objects::VelocityError>(
			+[]() { return Motor0.scalingFactors().velocity.toUser(Motor0.velocity()); });

		map.template setReadHandler<Objects::TargetVelocity>(
			+[]() { return Motor0.scalingFactors().velocity.toUser(Motor0.commandedVelocity()); });

		map.template setWriteHandler<Objects::TargetVelocity>(+[](int32_t value) {
			Motor0.setCommandedVelocity(Motor0.scalingFactors().velocity.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionDemandValue>(
			+[]() { return Motor0.commandedPosition(); });

		map.template setReadHandler<Objects::TargetPosition>(
			+[]() { return Motor0.scalingFactors().position.toUser(Motor0.receivedPosition()); });

		map.template setWriteHandler<Objects::TargetPosition>(+[](int32_t value) {
			Motor0.setReceivedPosition(Motor0.scalingFactors().position.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::VoltageCommand>(
			+[]() { return Motor0.commandedVoltage(); });

		map.template setWriteHandler<Objects::VoltageCommand>(+[](int16_t value) {
			Motor0.setCommandedVoltage(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::OutputVoltage>(
			+[]() { return Motor0.outputVoltage(); });

		map.template setReadHandler<Objects::ControlWord>(
			+[]() { return Motor0.control().value(); });

		map.template setWriteHandler<Objects::ControlWord>(+[](uint16_t value) {
			Motor0.control().update(value);
			Motor0.status().update(Motor0.control());
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionWindow>(
			+[]() { return Motor0.positionWindow(); });

		map.template setWriteHandler<Objects::PositionWindow>(+[](uint32_t value) {
			Motor0.setPositionWindow(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::StatusWord>(
			+[]() { return Motor0.status().status(); });

		map.template setReadHandler<Objects::PositionFactorNumerator>(
			+[]() { return Motor0.scalingFactors().position.numerator; });

		map.template setWriteHandler<Objects::PositionFactorNumerator>(+[](uint32_t value) {
			Motor0.scalingFactors().position.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionFactorDivisor>(
			+[]() { return Motor0.scalingFactors().position.divisor; });

		map.template setWriteHandler<Objects::PositionFactorDivisor>(+[](uint32_t value) {
			Motor0.scalingFactors().position.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::VelocityFactorNumerator>(
			+[]() { return Motor0.scalingFactors().velocity.numerator; });

		map.template setWriteHandler<Objects::VelocityFactorNumerator>(+[](uint32_t value) {
			Motor0.scalingFactors().velocity.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::VelocityFactorDivisor>(
			+[]() { return Motor0.scalingFactors().velocity.divisor; });

		map.template setWriteHandler<Objects::VelocityFactorDivisor>(+[](uint32_t value) {
			Motor0.scalingFactors().velocity.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::AccelerationFactorNumerator>(
			+[]() { return Motor0.scalingFactors().acceleration.numerator; });

		map.template setWriteHandler<Objects::AccelerationFactorNumerator>(+[](uint32_t value) {
			Motor0.scalingFactors().acceleration.numerator = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::AccelerationFactorDivisor>(
			+[]() { return Motor0.scalingFactors().acceleration.divisor; });

		map.template setWriteHandler<Objects::AccelerationFactorDivisor>(+[](uint32_t value) {
			Motor0.scalingFactors().acceleration.divisor = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::Polarity>(
			+[]() { return Motor0.scalingFactors().polarity; });

		map.template setWriteHandler<Objects::Polarity>(+[](uint8_t value) {
			Motor0.scalingFactors().polarity = value;
			return SdoErrorCode::NoError;
		});
	}
};
