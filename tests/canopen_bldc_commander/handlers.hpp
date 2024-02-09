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
		map.template setWriteHandler<StateObjects::UpdateTime, uint32_t>(+[](uint32_t value) {
			state.updateTime = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<PWMObjects::PWMCommand, int16_t>(
			+[]() { return state.commandedPWM; });

		map.template setWriteHandler<PWMObjects::PWMCommand, int16_t>(
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setWriteHandler<StateObjects::OutputPWM, int16_t>(+[](int16_t value) {
			if (state.outputPWM != value)
			{
				// MODM_LOG_INFO << "Received Output PWM of " << value << modm::endl;
				state.outputPWM = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::StatusWord, uint16_t>(+[](uint16_t value) {
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

		map.template setReadHandler<StateObjects::ControlWord, uint16_t>(
			+[]() { return state.control_.value(); });

		map.template setReadHandler<StateObjects::ModeOfOperation, int8_t>(
			+[]() { return (int8_t)state.currMode; });

		map.template setWriteHandler<StateObjects::ModeOfOperationDisplay, int8_t>(
			+[](int8_t value) {
				if ((int8_t)state.receivedMode != value)
				{
					MODM_LOG_INFO << "Received Mode " << value << modm::endl;
					state.receivedMode = (OperatingMode)value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::PositionActualValue, int32_t>(
			+[](int32_t value) {
				if (state.positionValue != value)
				{
					// MODM_LOG_INFO << "Received Output Position of " << value << modm::endl;
					state.positionValue = value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<PositionObjects::PositionDemandValue, int32_t>(
			+[](int32_t value) {
				if (state.posDemand != value) { state.posDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityDemandValue, int32_t>(
			+[](int32_t value) {
				if (state.velDemand != value) { state.velDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityError, int32_t>(+[](int32_t value) {
			if (state.velErrorValue != value) { state.velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::FilteredActualCurrent, float>(
			+[](float value) {
				if (state.currentValue != value) { state.currentValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<CurrentObjects::CommandedCurrent, float>(+[](float value) {
			if (state.commandedCurrent != value) { state.commandedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::CurrentError, float>(+[](float value) {
			if (state.currentErrorValue != value) { state.currentErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::MaxCharge, float>(+[](float value) {
			if (state.maxCharge != value) { state.maxCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::CurrentCharge, float>(+[](float value) {
			if (state.currentCharge != value) { state.currentCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<PositionObjects::FollowingErrorActualValue, int32_t>(
			+[](int32_t value) {
				if (state.posErrorValue != value) { state.posErrorValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::VelocityActualValue, int32_t>(
			+[](int32_t value) {
				if (state.velocityValue != value)
				{
					// MODM_LOG_INFO << "Received Output Velocity of " << value << modm::endl;
					state.velocityValue = value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setReadHandler<VelocityObjects::TargetVelocity, int32_t>(
			+[]() { return state.targetSpeed; });

		map.template setWriteHandler<VelocityObjects::TargetVelocity, int32_t>(+[](int32_t value) {
			if (state.targetSpeed != value) { state.targetSpeed = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::OrientedCurrent, float>(+[](int32_t value) {
			if (state.orientedCurrent != value) { state.orientedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::OrientedCurrentAngleDiff, float>(
			+[](int32_t value) {
				if (state.orientedCurrentAngleDiff != value)
				{
					state.orientedCurrentAngleDiff = value;
				}
				return SdoErrorCode::NoError;
			});
	}
};