#pragma once

#include <modm-canopen/canopen_device.hpp>
#include "motor.hpp"

struct Objects
{
	static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
	static constexpr modm_canopen::Address PositionActualValue{0x6064, 0};
	static constexpr modm_canopen::Address VelocityActualValue{0x606C, 0};
	static constexpr modm_canopen::Address TargetVelocity{0x60FF, 0};
	static constexpr modm_canopen::Address EnableMotor{0x2001, 0};
	static constexpr modm_canopen::Address VoltageCommand{0x2002, 0};
	static constexpr modm_canopen::Address OutputVoltage{0x2003, 0};
};

struct CanOpenHandlers
{
	template<typename ObjectDictionary>
	constexpr void registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
	{
		using modm_canopen::SdoErrorCode;

		map.template setReadHandler<Objects::ModeOfOperation>(
			+[](){ return int8_t(Motor0.mode()); });

		map.template setWriteHandler<Objects::ModeOfOperation>(
			+[](int8_t value)
			{
				const bool valid = (value == int8_t(ControlMode::Disabled)) ||
					(value == int8_t(ControlMode::Voltage)) ||
					(value == int8_t(ControlMode::Velocity));

				if (valid) {
					Motor0.setMode(static_cast<ControlMode>(value));
					return SdoErrorCode::NoError;
				} else {
					return SdoErrorCode::InvalidValue;
				}
			});


		map.template setReadHandler<Objects::PositionActualValue>(
			+[](){ return Motor0.position(); });

		map.template setReadHandler<Objects::VelocityActualValue>(
			+[](){ return Motor0.velocity(); });


		map.template setReadHandler<Objects::TargetVelocity>(
			+[](){ return Motor0.commandedVelocity(); });

		map.template setWriteHandler<Objects::TargetVelocity>(
			+[](int32_t value)
			{
				Motor0.setCommandedVelocity(value);
				return SdoErrorCode::NoError;
			});


		map.template setReadHandler<Objects::EnableMotor>(
			+[](){ return (uint8_t) Motor0.isEnabled(); });

		map.template setWriteHandler<Objects::EnableMotor>(
			+[](uint8_t value)
			{
				Motor0.setEnabled(value > 0);
				return SdoErrorCode::NoError;
			});


		map.template setReadHandler<Objects::VoltageCommand>(
			+[](){ return Motor0.commandedVoltage(); });

		map.template setWriteHandler<Objects::VoltageCommand>(
			+[](int16_t value)
			{
				Motor0.setCommandedVoltage(value);
				return SdoErrorCode::NoError;
			});

		map.template setReadHandler<Objects::OutputVoltage>(
			+[](){ return Motor0.outputVoltage(); });
	}
};
