#pragma once
#include <modm-canopen/canopen_device.hpp>
#include "motor_control.hpp"
#include "canopen_objects.hpp"

struct VelocityHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;
		map.template setReadHandler<Objects::VelocityActualValue>(+[]() {
			return MotorControl0.scalingFactors().velocity.toUser(MotorControl0.velocity());
		});

		map.template setReadHandler<Objects::VelocityError>(+[]() {
			return MotorControl0.scalingFactors().velocity.toUser(MotorControl0.velocityError());
		});

		map.template setReadHandler<Objects::TargetVelocity>(+[]() {
			return MotorControl0.scalingFactors().velocity.toUser(
				MotorControl0.commandedVelocity());
		});

		map.template setWriteHandler<Objects::TargetVelocity>(+[](int32_t value) {
			MotorControl0.setCommandedVelocity(
				MotorControl0.scalingFactors().velocity.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kP>(+[](float value) {
			auto params = MotorControl0.getVelocityPidParams();
			params.setKp(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kI>(+[](float value) {
			auto params = MotorControl0.getVelocityPidParams();
			params.setKi(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kD>(+[](float value) {
			auto params = MotorControl0.getVelocityPidParams();
			params.setKd(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_MaxErrorSum>(+[](float value) {
			auto params = MotorControl0.getVelocityPidParams();
			params.setMaxErrorSum(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});
	}
};
