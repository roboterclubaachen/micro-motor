#ifndef MOTOR_CONTROL_HPP
#error "Do not include this file directly, use motor_control.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template<typename... Modes>
template<typename Device, typename MessageCallback, typename First>
bool
MotorControl<Modes...>::updateMode(MessageCallback&& cb)
{
	if (First::applicable(state_))
		return First::template update<Device, MessageCallback>(state_,
															   std::forward<MessageCallback>(cb));
	return true;
}

template<typename... Modes>
template<typename Device, typename MessageCallback, typename First, typename Second,
		 typename... Rest>
bool
MotorControl<Modes...>::updateMode(MessageCallback&& cb)
{
	auto out = false;
	if (First::applicable(state_))
		out = First::template update<Device, MessageCallback>(state_,
															  std::forward<MessageCallback>(cb));
	return updateMode<Device, MessageCallback, Second, Rest...>(
			   std::forward<MessageCallback>(cb)) &&
		   out;
}

template<typename... Modes>
template<typename Device, typename MessageCallback>
bool
MotorControl<Modes...>::update(MessageCallback&& cb)
{
	auto now = modm::chrono::micro_clock::now();
	state_.updateTime_.update((now - state_.lastUpdate_).count());
	state_.lastUpdate_ = now;
	Device::setValueChanged(StateObjects::UpdateTime);

	const auto newVelocity_ = state_.actualPosition_ - state_.lastPosition_;
	state_.lastPosition_ = state_.actualPosition_;
	state_.actualVelocity_.update(newVelocity_);

	Device::setValueChanged(StateObjects::PositionInternalValue);
	Device::setValueChanged(StateObjects::PositionActualValue);
	Device::setValueChanged(StateObjects::VelocityActualValue);

	bool value = false;
	if (state_.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
		state_.mode_ == OperatingMode::Disabled)
	{
		state_.enableMotor_ = false;
		state_.outputPWM_ = 0;
		value = true;
	} else
	{
		state_.enableMotor_ = true;
	}
	auto temp = updateMode<Device, MessageCallback, Modes...>(std::forward<MessageCallback>(cb));
	value = value || temp;
	state_.status_.setBit<StatusBits::VoltagePresent>(state_.outputPWM_ != 0);
	Device::setValueChanged(StateObjects::OutputPWM);
	Device::setValueChanged(StateObjects::StatusWord);
	Device::setValueChanged(
		StateObjects::ModeOfOperationDisplay);  // TODO move to where this actually happens
	return value;
}

template<typename... Modes>
template<typename ObjectDictionary>
constexpr void
MotorControl<Modes...>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<StateObjects::UpdateTime>(
		+[]() { return uint32_t(state_.updateTime_.getValue()); });

	map.template setReadHandler<StateObjects::ModeOfOperation>(
		+[]() { return int8_t(state_.mode_); });

	map.template setReadHandler<StateObjects::ModeOfOperationDisplay>(
		+[]() { return int8_t(state_.mode_); });

	map.template setWriteHandler<StateObjects::ModeOfOperation>(+[](int8_t value) {
		const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
						   (value == int8_t(OperatingMode::Voltage)) ||
						   (value == int8_t(OperatingMode::Velocity)) ||
						   (value == int8_t(OperatingMode::Position));

		if (valid)
		{
			auto newMode = (static_cast<OperatingMode>(value));
			if (state_.mode_ != newMode)
			{
				state_.mode_ = newMode;
				MODM_LOG_INFO << "Set operating mode to "
							  << modm_canopen::cia402::operatingModeToString(state_.mode_)
							  << modm::endl;
			}
			return SdoErrorCode::NoError;
		} else
		{
			return SdoErrorCode::InvalidValue;
		}
	});

	map.template setReadHandler<StateObjects::ControlWord>(
		+[]() { return state_.control_.value(); });

	map.template setWriteHandler<StateObjects::ControlWord>(+[](uint16_t value) {
		state_.control_.update(value);
		state_.status_.update(state_.control_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::StatusWord>(
		+[]() { return state_.status_.status(); });

	map.template setReadHandler<StateObjects::PositionActualValue>(
		+[]() { return state_.scalingFactors_.position.toUser(state_.actualPosition_); });

	map.template setReadHandler<StateObjects::PositionInternalValue>(
		+[]() { return state_.actualPosition_; });

	map.template setReadHandler<StateObjects::VelocityActualValue>(+[]() {
		return state_.scalingFactors_.velocity.toUser(state_.actualVelocity_.getValue());
	});

	map.template setReadHandler<StateObjects::OutputPWM>(+[]() { return state_.outputPWM_; });

	map.template setReadHandler<StateObjects::PositionFactorNumerator>(
		+[]() { return state_.scalingFactors_.position.numerator; });

	map.template setWriteHandler<StateObjects::PositionFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.position.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::PositionFactorDivisor>(
		+[]() { return state_.scalingFactors_.position.divisor; });

	map.template setWriteHandler<StateObjects::PositionFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.position.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::VelocityFactorNumerator>(
		+[]() { return state_.scalingFactors_.velocity.numerator; });

	map.template setWriteHandler<StateObjects::VelocityFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.velocity.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::VelocityFactorDivisor>(
		+[]() { return state_.scalingFactors_.velocity.divisor; });

	map.template setWriteHandler<StateObjects::VelocityFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.velocity.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::AccelerationFactorNumerator>(
		+[]() { return state_.scalingFactors_.acceleration.numerator; });

	map.template setWriteHandler<StateObjects::AccelerationFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.acceleration.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::AccelerationFactorDivisor>(
		+[]() { return state_.scalingFactors_.acceleration.divisor; });

	map.template setWriteHandler<StateObjects::AccelerationFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.acceleration.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::Polarity>(
		+[]() { return state_.scalingFactors_.getPolarity(); });

	map.template setWriteHandler<StateObjects::Polarity>(+[](uint8_t value) {
		state_.scalingFactors_.setPolarity(value);
		return SdoErrorCode::NoError;
	});

	(Modes::template registerHandlers<ObjectDictionary, state_>(map), ...);
}
