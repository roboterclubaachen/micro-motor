#ifndef VELOCITY_PROTOCOL_HPP
#error "Do not include this file directly, use velocity_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

bool
VelocityProtocol::update(MotorState& state)
{

	if (state.mode_ == OperatingMode::Velocity ||
		state.control_.isSet<CommandBits::ChangeImmediately>() || velocityError_ == 0)
	{
		commandedVelocity_ = receivedVelocity_;
	}

	// TODO implement profile acceleration
	if (!state.control_.isSet<CommandBits::Halt>())
	{
		velocityError_ = (commandedVelocity_ - state.actualVelocity_.getValue());
	} else
	{
		velocityError_ = -state.actualVelocity_.getValue();
	}

	velocityPid_.update(velocityError_);
	state.outputPWM_ = velocityPid_.getValue();

	state.status_.setBit<StatusBits::TargetReached>(velocityError_ == 0);
	state.status_.setBit<StatusBits::SpeedZero>(
		state.actualVelocity_.getValue() ==
		0);  // TODO implement velocity Threshold (for zero and no speed indication)
	// TODO implement max slippage
	return true;
}

int16_t
VelocityProtocol::updatePid(int32_t commandedVelocity, int32_t actualVelocity)
{
	commandedVelocity_ = commandedVelocity;
	velocityError_ = commandedVelocity_ - actualVelocity;
	velocityPid_.update(velocityError_);
	return velocityPid_.getValue();
}

template<typename ObjectDictionary, const MotorState& state>
constexpr void
VelocityProtocol::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<VelocityObjects::VelocityDemandValue>(
		+[]() { return state.scalingFactors_.velocity.toUser(commandedVelocity_); });

	map.template setReadHandler<VelocityObjects::VelocityActualValue>(
		+[]() { return state.scalingFactors_.velocity.toUser(state.actualVelocity_.getValue()); });

	map.template setReadHandler<VelocityObjects::VelocityError>(
		+[]() { return state.scalingFactors_.velocity.toUser(velocityError_); });

	map.template setReadHandler<VelocityObjects::TargetVelocity>(
		+[]() { return state.scalingFactors_.velocity.toUser(receivedVelocity_); });

	map.template setWriteHandler<VelocityObjects::TargetVelocity>(+[](int32_t value) {
		receivedVelocity_ = state.scalingFactors_.velocity.toInternal(value);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kP>(+[](float value) {
		velocityPidParameters_.setKp(value);
		velocityPid_.setParameter(velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kI>(+[](float value) {
		velocityPidParameters_.setKi(value);
		velocityPid_.setParameter(velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kD>(+[](float value) {
		velocityPidParameters_.setKd(value);
		velocityPid_.setParameter(velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_MaxErrorSum>(+[](float value) {
		velocityPidParameters_.setMaxErrorSum(value);
		velocityPid_.setParameter(velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<VelocityObjects::ProfileAcceleration>(
		+[]() { return state.scalingFactors_.acceleration.toUser(profileAcceleration_); });

	map.template setWriteHandler<VelocityObjects::ProfileAcceleration>(+[](int32_t value) {
		profileAcceleration_ = state.scalingFactors_.acceleration.toInternal(value);
		return SdoErrorCode::NoError;
	});
}
