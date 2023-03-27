#ifndef VELOCITY_PROTOCOL_HPP
#error "Do not include this file directly, use velocity_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>
#include <algorithm>

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

template<typename Device, typename MessageCallback>
bool
VelocityProtocol::update(MotorState& state, MessageCallback&&)
{

	if (state.mode_ == OperatingMode::Velocity ||
		state.control_.isSet<CommandBits::ChangeImmediately>() || velocityError_ == 0)
	{
		commandedVelocity_ = receivedVelocity_;
	}

	Device::setValueChanged(VelocityObjects::VelocityDemandValue);

	// TODO implement profile acceleration
	if (!state.control_.isSet<CommandBits::Halt>())
	{
		velocityError_ = (commandedVelocity_ - state.actualVelocity_.getValue());
	} else
	{
		velocityError_ = -state.actualVelocity_.getValue();
	}

	Device::setValueChanged(VelocityObjects::VelocityError);

	velocityPid_.update(velocityError_, state.outputPWM_ > profileAcceleration_);
	state.outputPWM_ =
		std::clamp((int32_t)velocityPid_.getValue(), -profileAcceleration_, profileAcceleration_);

	state.status_.setBit<StatusBits::TargetReached>(velocityError_ == 0);
	state.status_.setBit<StatusBits::SpeedZero>(
		state.actualVelocity_.getValue() ==
		0);  // TODO implement velocity Threshold (for zero and no speed indication)
	// TODO implement max slippage
	return true;
}

int16_t
VelocityProtocol::doPositionUpdate(int32_t commandedVelocity, const MotorState& state)
{
	commandedVelocity_ = commandedVelocity;
	velocityError_ = commandedVelocity_ - state.actualVelocity_.getValue();
	velocityPid_.update(velocityError_, state.outputPWM_ > profileAcceleration_);
	return (int16_t)std::clamp((int32_t)velocityPid_.getValue(), -profileAcceleration_,
							   profileAcceleration_);
}

int16_t
VelocityProtocol::doQuickStopUpdate(int32_t commandedDeceleration, const MotorState& state)
{
	commandedVelocity_ = 0;
	velocityError_ = commandedVelocity_ - state.actualVelocity_.getValue();
	velocityPid_.update(velocityError_, state.outputPWM_ > commandedDeceleration);
	return (int16_t)std::clamp((int32_t)velocityPid_.getValue(), -commandedDeceleration,
							   commandedDeceleration);
}

template<typename ObjectDictionary, const MotorState& state>
constexpr void
VelocityProtocol::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<VelocityObjects::VelocityDemandValue>(
		+[]() { return state.scalingFactors_.velocity.toUser(commandedVelocity_); });

	map.template setReadHandler<VelocityObjects::VelocityError>(
		+[]() { return state.scalingFactors_.velocity.toUser(velocityError_); });

	map.template setReadHandler<VelocityObjects::TargetVelocity>(
		+[]() { return state.scalingFactors_.velocity.toUser(receivedVelocity_); });

	map.template setWriteHandler<VelocityObjects::TargetVelocity>(+[](int32_t value) {
		receivedVelocity_ = state.scalingFactors_.velocity.toInternal(value);
		MODM_LOG_INFO << "Set Target Velocity to " << receivedVelocity_ << modm::endl;
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
