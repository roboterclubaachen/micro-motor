#pragma once

#include "state.hpp"

#include <librobots2/motor-canopen/canopen_objects.hpp>
#include <modm-canopen/master/canopen_master.hpp>
#include <modm-canopen/master/sdo_client.hpp>
#include <modm-canopen/generated/micro-motor_od.hpp>

using modm_canopen::SdoErrorCode;

struct Test
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(uint8_t, modm_canopen::HandlerMapRT<ObjectDictionary>& map)
	{
		map.template setWriteHandler<uint32_t>(StateObjects::UpdateTime, +[](uint32_t value) {
			state.updateTime = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<int16_t>(PWMObjects::PWMCommand,
			+[]() { return state.commandedPWM; });

		map.template setWriteHandler<int16_t>(PWMObjects::PWMCommand,
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setWriteHandler<int16_t>(StateObjects::OutputPWM,+[](int16_t value) {
			if (state.outputPWM != value)
			{
				// MODM_LOG_INFO << "Received Output PWM of " << value << modm::endl;
				state.outputPWM = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<uint16_t>(StateObjects::StatusWord,+[](uint16_t value) {
			state.state_.set(value);
			if (state.state_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() &&
				!state.targetReached)
			{
				MODM_LOG_INFO << "Target Reached!" << modm::endl;
				state.targetReached = true;
			}
			if (!state.state_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() &&
				state.targetReached)
			{
				MODM_LOG_INFO << "Target Lost!" << modm::endl;
				state.targetReached = false;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<uint16_t>(StateObjects::ControlWord,
			+[]() { return state.control_.value(); });

		map.template setReadHandler<int8_t>(StateObjects::ModeOfOperation,
			+[]() { return (int8_t)state.currMode; });

		map.template setWriteHandler<int8_t>(StateObjects::ModeOfOperationDisplay,
			+[](int8_t value) {
				if ((int8_t)state.receivedMode != value)
				{
					MODM_LOG_INFO << "Received Mode " << value << modm::endl;
					state.receivedMode = (OperatingMode)value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<int32_t>(StateObjects::PositionActualValue,
			+[](int32_t value) {
				if (state.positionValue != value)
				{
					// MODM_LOG_INFO << "Received Output Position of " << value << modm::endl;
					state.positionValue = value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<int32_t>(PositionObjects::PositionDemandValue,
			+[](int32_t value) {
				if (state.posDemand != value) { state.posDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<int32_t>(VelocityObjects::VelocityDemandValue,
			+[](int32_t value) {
				if (state.velDemand != value) { state.velDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<int32_t>(VelocityObjects::VelocityError,+[](int32_t value) {
			if (state.velErrorValue != value) { state.velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(CurrentObjects::FilteredActualCurrent,
			+[](float value) {
				if (state.currentValue != value) { state.currentValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<float>(CurrentObjects::CommandedCurrent,+[](float value) {
			if (state.commandedCurrent != value) { state.commandedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(CurrentObjects::CurrentError,+[](float value) {
			if (state.currentErrorValue != value) { state.currentErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(StateObjects::MaxCharge,+[](float value) {
			if (state.maxCharge != value) { state.maxCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(StateObjects::CurrentCharge,+[](float value) {
			if (state.currentCharge != value) { state.currentCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<int32_t>(PositionObjects::FollowingErrorActualValue,
			+[](int32_t value) {
				if (state.posErrorValue != value) { state.posErrorValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<int32_t>(StateObjects::VelocityActualValue,
			+[](int32_t value) {
				if (state.velocityValue != value)
				{
					// MODM_LOG_INFO << "Received Output Velocity of " << value << modm::endl;
					state.velocityValue = value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setReadHandler<int32_t>(VelocityObjects::TargetVelocity,
			+[]() { return state.targetSpeed; });

		map.template setWriteHandler<int32_t>(VelocityObjects::TargetVelocity,+[](int32_t value) {
			if (state.targetSpeed != value) { state.targetSpeed = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(StateObjects::OrientedCurrent,+[](int32_t value) {
			if (state.orientedCurrent != value) { state.orientedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<float>(StateObjects::OrientedCurrentAngleDiff,
			+[](int32_t value) {
				if (state.orientedCurrentAngleDiff != value)
				{
					state.orientedCurrentAngleDiff = value;
				}
				return SdoErrorCode::NoError;
			});
		map.template setWriteHandler<uint16_t>(EncoderObjects::EncoderValue,+[](uint16_t value) {
			static uint16_t oldVal = 0;
			if (oldVal != value)
			{
				oldVal = value;
				MODM_LOG_INFO << "Enc " << value << modm::endl;
			}
			return SdoErrorCode::NoError;
		});
	}
};