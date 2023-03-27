#ifndef POSITION_PROTOCOL_HPP
#error "Do not include this file directly, use position_protocol.hpp instead"
#endif
#include <modm/debug/logger.hpp>

template<typename VelocityProtocol>
template<typename Device, typename MessageCallback>
bool
PositionProtocol<VelocityProtocol>::update(MotorState& state, MessageCallback&&)
{
	if (state.control_.isSet<CommandBits::NewSetPoint>())
	{
		nextPosition_ = receivedPosition_;
		nextPositionIsNew_ = true;
		state.control_.setBit<CommandBits::NewSetPoint>(false);
	}

	if ((positionError_ == 0 && nextPositionIsNew_) ||
		state.control_.isSet<CommandBits::ChangeImmediately>())
	{
		nextPositionIsNew_ = false;
		if (state.control_.isSet<CommandBits::IsRelative>())
		{
			commandedPosition_ += nextPosition_;
			MODM_LOG_INFO << "Updated Target Position relative!" << modm::endl;
		} else if (commandedPosition_ != nextPosition_)
		{
			commandedPosition_ = nextPosition_;
			MODM_LOG_INFO << "Updated Target Position absolute!" << modm::endl;
		}
		Device::setValueChanged(PositionObjects::PositionDemandValue);
	}

	positionError_ = commandedPosition_ - state.actualPosition_;
	Device::setValueChanged(PositionObjects::FollowingErrorActualValue);

	positionPid_.update(positionError_);
	state.outputPWM_ = VelocityProtocol::doPositionUpdate(positionPid_.getValue(), state);

	Device::setValueChanged(VelocityObjects::VelocityError);
	Device::setValueChanged(VelocityObjects::VelocityDemandValue);

	if ((uint32_t)std::abs(positionError_) <= positionWindow_)
	{
		if (inPositionWindow_ < positionWindowTime_) inPositionWindow_++;
	} else
	{
		inPositionWindow_ = 0;
	}

	state.status_.setBit<StatusBits::TargetReached>(inPositionWindow_ >= positionWindowTime_);
	return true;
}

template<typename VelocityProtocol>
template<typename ObjectDictionary, const MotorState& state>
constexpr void
PositionProtocol<VelocityProtocol>::registerHandlers(
	modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<PositionObjects::FollowingErrorActualValue>(
		+[]() { return state.scalingFactors_.position.toUser(positionError_); });

	map.template setReadHandler<PositionObjects::PositionDemandValue>(
		+[]() { return state.scalingFactors_.position.toUser(commandedPosition_); });

	map.template setReadHandler<PositionObjects::TargetPosition>(
		+[]() { return state.scalingFactors_.position.toUser(receivedPosition_); });

	map.template setWriteHandler<PositionObjects::TargetPosition>(+[](int32_t value) {
		receivedPosition_ = state.scalingFactors_.position.toInternal(value);
		MODM_LOG_INFO << "Set Target Position to " << receivedPosition_ << modm::endl;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<PositionObjects::PositionWindow>(
		+[]() { return state.scalingFactors_.position.toUser(positionWindow_); });

	map.template setWriteHandler<PositionObjects::PositionWindow>(+[](uint32_t value) {
		positionWindow_ = state.scalingFactors_.position.toInternal(value);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kP>(+[](float value) {
		positionPidParameters_.setKp(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kI>(+[](float value) {
		positionPidParameters_.setKi(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kD>(+[](float value) {
		positionPidParameters_.setKd(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_MaxErrorSum>(+[](float value) {
		positionPidParameters_.setMaxErrorSum(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});
}
