#pragma once
#include <modm-canopen/canopen_device.hpp>
#include "motor_control.hpp"
#include "canopen_objects.hpp"

struct PositionHandlers
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
	{
		using modm_canopen::SdoErrorCode;
		map.template setReadHandler<Objects::PositionActualValue>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.position());
		});

		map.template setReadHandler<Objects::PositionInternalValue>(
			+[]() { return MotorControl0.position(); });

		map.template setReadHandler<Objects::PositionError>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.positionError());
		});

		map.template setReadHandler<Objects::PositionDemandValue>(
			+[]() { return MotorControl0.commandedPosition(); });

		map.template setReadHandler<Objects::TargetPosition>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.receivedPosition());
		});

		map.template setWriteHandler<Objects::TargetPosition>(+[](int32_t value) {
			MotorControl0.setCommandedPosition(
				MotorControl0.scalingFactors().position.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionWindow>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.positionWindow());
		});

		map.template setWriteHandler<Objects::PositionWindow>(+[](uint32_t value) {
			MotorControl0.setPositionWindow(
				MotorControl0.scalingFactors().position.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::PositionPID_kP>(+[](float value) {
			MODM_LOG_DEBUG << "Received kP ==" << value << modm::endl;
			auto params = MotorControl0.getPositionPidParams();
			params.setKp(value);
			MotorControl0.setPositionPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::PositionPID_kI>(+[](float value) {
			MODM_LOG_DEBUG << "Received kI ==" << value << modm::endl;
			auto params = MotorControl0.getPositionPidParams();
			params.setKi(value);
			MotorControl0.setPositionPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::PositionPID_kD>(+[](float value) {
			MODM_LOG_DEBUG << "Received kD ==" << value << modm::endl;
			auto params = MotorControl0.getPositionPidParams();
			params.setKd(value);
			MotorControl0.setPositionPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::PositionPID_MaxErrorSum>(+[](float value) {
			MODM_LOG_DEBUG << "Received Max Error Sum ==" << value << modm::endl;
			auto params = MotorControl0.getPositionPidParams();
			params.setMaxErrorSum(value);
			MotorControl0.setPositionPidParams(params);
			return SdoErrorCode::NoError;
		});
	}
};
