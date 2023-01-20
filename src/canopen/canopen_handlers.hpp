#pragma once
#include <modm-canopen/canopen_device.hpp>
#include <modm/debug/logger.hpp>

#include "motor_control.hpp"

#include "canopen_objects.hpp"

struct CanOpenHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;

		map.template setReadHandler<Objects::ModeOfOperation>(
			+[]() { return int8_t(MotorControl0.mode()); });

		map.template setReadHandler<Objects::ModeOfOperationDisplay>(
			+[]() { return int8_t(MotorControl0.mode()); });

		map.template setWriteHandler<Objects::ModeOfOperation>(+[](int8_t value) {
			const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
							   (value == int8_t(OperatingMode::Voltage)) ||
							   (value == int8_t(OperatingMode::Velocity)) ||
							   (value == int8_t(OperatingMode::Position));

			if (valid)
			{
				MotorControl0.setMode(static_cast<OperatingMode>(value));
				return SdoErrorCode::NoError;
			} else
			{
				return SdoErrorCode::InvalidValue;
			}
		});

		map.template setReadHandler<Objects::ControlWord>(
			+[]() { return MotorControl0.control().value(); });

		map.template setWriteHandler<Objects::ControlWord>(+[](uint16_t value) {
			MotorControl0.control().update(value);
			MotorControl0.status().update(MotorControl0.control());
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::StatusWord>(
			+[]() { return MotorControl0.status().status(); });
	}
};
