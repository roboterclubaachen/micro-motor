#pragma once
#include <modm-canopen/canopen_device.hpp>
#include "motor_control.hpp"
#include "canopen_objects.hpp"

struct PWMHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;

		map.template setReadHandler<Objects::PWMCommand>(
			+[]() { return MotorControl0.commandedPWM(); });

		map.template setWriteHandler<Objects::PWMCommand>(+[](int16_t value) {
			MotorControl0.setCommandedPWM(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::OutputPWM>(
			+[]() { return MotorControl0.outputPWM(); });
	}
};
