
#pragma once

#include <librobots2/motor-canopen/canopen_objects.hpp>
#include <modm-canopen/master/canopen_master.hpp>
#include <modm-canopen/master/sdo_client.hpp>
#include <modm-canopen/generated/micro-motor_od.hpp>

#include "motor_state.hpp"

using modm_canopen::Address;
using modm_canopen::SdoErrorCode;

struct CanopenCallbacks
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(uint8_t, modm_canopen::HandlerMapRT<ObjectDictionary>& map)
	{
		map.template setWriteHandler<StateObjects::UpdateTime, uint32_t>(+[](uint32_t value) {
			state_.updateTime = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<PWMObjects::PWMCommand, int16_t>(
			+[]() { return state_.commandedPWM; });

		map.template setWriteHandler<PWMObjects::PWMCommand, int16_t>(
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setWriteHandler<StateObjects::OutputPWM, int16_t>(+[](int16_t value) {
			if (state_.outputPWM != value) { state_.outputPWM = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::StatusWord, uint16_t>(+[](uint16_t value) {
			state_.stateMachine_.set(value);
			if (state_.stateMachine_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() &&
				!state_.targetReached)
			{
				MODM_LOG_INFO << "Target Reached!" << modm::endl;
				state_.targetReached = true;
			}
			if (!state_.stateMachine_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() &&
				state_.targetReached)
			{
				MODM_LOG_INFO << "Target Lost!" << modm::endl;
				state_.targetReached = false;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<StateObjects::ControlWord, uint16_t>(
			+[]() { return state_.control_.value(); });

		map.template setReadHandler<StateObjects::ModeOfOperation, int8_t>(
			+[]() { return (int8_t)state_.currMode; });

		map.template setWriteHandler<StateObjects::ModeOfOperationDisplay, int8_t>(
			+[](int8_t value) {
				if ((int8_t)state_.receivedMode != value)
				{
					state_.receivedMode = (OperatingMode)value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::PositionActualValue, int32_t>(
			+[](int32_t value) {
				if (state_.positionValue != value) { state_.positionValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<PositionObjects::PositionDemandValue, int32_t>(
			+[](int32_t value) {
				if (state_.posDemand != value) { state_.posDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityDemandValue, int32_t>(
			+[](int32_t value) {
				if (state_.velDemand != value) { state_.velDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityError, int32_t>(+[](int32_t value) {
			if (state_.velErrorValue != value) { state_.velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::FilteredActualCurrent, float>(
			+[](float value) {
				if (state_.currentValue != value) { state_.currentValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<CurrentObjects::CommandedCurrent, float>(+[](float value) {
			if (state_.commandedCurrent != value) { state_.commandedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::CurrentError, float>(+[](float value) {
			if (state_.currentErrorValue != value) { state_.currentErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<PositionObjects::FollowingErrorActualValue, int32_t>(
			+[](int32_t value) {
				if (state_.posErrorValue != value) { state_.posErrorValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::VelocityActualValue, int32_t>(
			+[](int32_t value) {
				if (state_.velocityValue != value) { state_.velocityValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setReadHandler<VelocityObjects::TargetVelocity, int32_t>(
			+[]() { return state_.targetSpeed; });

		map.template setWriteHandler<VelocityObjects::TargetVelocity, int32_t>(+[](int32_t value) {
			if (state_.targetSpeed != value) { state_.targetSpeed = value; }
			return SdoErrorCode::NoError;
		});
	}
};