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

		map.template setReadHandler<Objects::PositionActualValue>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.position());
		});

		map.template setReadHandler<Objects::VelocityActualValue>(+[]() {
			return MotorControl0.scalingFactors().velocity.toUser(MotorControl0.velocity());
		});

		map.template setReadHandler<Objects::VelocityError>(+[]() {
			return MotorControl0.scalingFactors().velocity.toUser(MotorControl0.velocityError());
		});

		map.template setReadHandler<Objects::PositionError>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.positionError());
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

		map.template setReadHandler<Objects::PositionDemandValue>(
			+[]() { return MotorControl0.commandedPosition(); });

		map.template setReadHandler<Objects::TargetPosition>(+[]() {
			return MotorControl0.scalingFactors().position.toUser(MotorControl0.receivedPosition());
		});

		map.template setWriteHandler<Objects::TargetPosition>(+[](int32_t value) {
			MotorControl0.setReceivedPosition(
				MotorControl0.scalingFactors().position.toInternal(value));
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PWMCommand>(
			+[]() { return MotorControl0.commandedPWM(); });

		map.template setWriteHandler<Objects::PWMCommand>(+[](int16_t value) {
			MotorControl0.setCommandedPWM(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::OutputPWM>(
			+[]() { return MotorControl0.outputPWM(); });

		map.template setReadHandler<Objects::ControlWord>(
			+[]() { return MotorControl0.control().value(); });

		map.template setWriteHandler<Objects::ControlWord>(+[](uint16_t value) {
			MotorControl0.control().update(value);
			MotorControl0.status().update(MotorControl0.control());
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PositionWindow>(
			+[]() { return MotorControl0.positionWindow(); });

		map.template setWriteHandler<Objects::PositionWindow>(+[](uint32_t value) {
			MotorControl0.setPositionWindow(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::StatusWord>(
			+[]() { return MotorControl0.status().status(); });

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
			+[]() { return MotorControl0.scalingFactors().polarity; });

		map.template setWriteHandler<Objects::Polarity>(+[](uint8_t value) {
			MotorControl0.scalingFactors().polarity = value;
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kP>(+[](float value) {
			MODM_LOG_DEBUG << "Received kP ==" << value << modm::endl;
			auto params = MotorControl0.getVelocityPidParams();
			params.setKp(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kI>(+[](float value) {
			MODM_LOG_DEBUG << "Received kI ==" << value << modm::endl;
			auto params = MotorControl0.getVelocityPidParams();
			params.setKi(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_kD>(+[](float value) {
			MODM_LOG_DEBUG << "Received kD ==" << value << modm::endl;
			auto params = MotorControl0.getVelocityPidParams();
			params.setKd(value);
			MotorControl0.setVelocityPidParams(params);
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityPID_MaxErrorSum>(+[](float value) {
			MODM_LOG_DEBUG << "Received Max Error Sum ==" << value << modm::endl;
			auto params = MotorControl0.getVelocityPidParams();
			params.setMaxErrorSum(value);
			MotorControl0.setVelocityPidParams(params);
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
