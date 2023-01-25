#ifndef MOTOR_CONTROL_HPP
#error "Do not include this file directly, use motor_control.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template<typename... Modes>
template<typename First>
bool
MotorControl<Modes...>::updateMode()
{
	if (First::applicable(state_)) return First::update(state_);
	return false;
}

template<typename... Modes>
template<typename First, typename Second, typename... Rest>
bool
MotorControl<Modes...>::updateMode()
{
	if (First::applicable(state_)) return First::update(state_);
	return updateMode<Second, Rest...>();
}

template<typename... Modes>
bool
MotorControl<Modes...>::update()
{
	const auto newVelocity_ = state_.actualPosition_ - state_.lastPosition_;
	state_.lastPosition_ = state_.actualPosition_;
	state_.actualVelocity_.update(newVelocity_);

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
		value = updateMode<Modes...>();
	}
	state_.status_.setBit<StatusBits::VoltagePresent>(state_.outputPWM_ != 0);
	return value;
}

template<typename... Modes>
template<typename ObjectDictionary>
constexpr void
MotorControl<Modes...>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<CanopenObjects::ModeOfOperation>(
		+[]() { return int8_t(state_.mode_); });

	map.template setReadHandler<CanopenObjects::ModeOfOperationDisplay>(
		+[]() { return int8_t(state_.mode_); });

	map.template setWriteHandler<CanopenObjects::ModeOfOperation>(+[](int8_t value) {
		const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
						   (value == int8_t(OperatingMode::Voltage)) ||
						   (value == int8_t(OperatingMode::Velocity)) ||
						   (value == int8_t(OperatingMode::Position));

		if (valid)
		{
			state_.mode_ = (static_cast<OperatingMode>(value));
			return SdoErrorCode::NoError;
		} else
		{
			return SdoErrorCode::InvalidValue;
		}
	});

	map.template setReadHandler<CanopenObjects::ControlWord>(
		+[]() { return state_.control_.value(); });

	map.template setWriteHandler<CanopenObjects::ControlWord>(+[](uint16_t value) {
		state_.control_.update(value);
		state_.status_.update(state_.control_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<CanopenObjects::StatusWord>(
		+[]() { return state_.status_.status(); });

	map.template setReadHandler<FactorObjects::PositionFactorNumerator>(
		+[]() { return state_.scalingFactors_.position.numerator; });

	map.template setWriteHandler<FactorObjects::PositionFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.position.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::PositionFactorDivisor>(
		+[]() { return state_.scalingFactors_.position.divisor; });

	map.template setWriteHandler<FactorObjects::PositionFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.position.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::VelocityFactorNumerator>(
		+[]() { return state_.scalingFactors_.velocity.numerator; });

	map.template setWriteHandler<FactorObjects::VelocityFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.velocity.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::VelocityFactorDivisor>(
		+[]() { return state_.scalingFactors_.velocity.divisor; });

	map.template setWriteHandler<FactorObjects::VelocityFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.velocity.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::AccelerationFactorNumerator>(
		+[]() { return state_.scalingFactors_.acceleration.numerator; });

	map.template setWriteHandler<FactorObjects::AccelerationFactorNumerator>(+[](uint32_t value) {
		state_.scalingFactors_.acceleration.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::AccelerationFactorDivisor>(
		+[]() { return state_.scalingFactors_.acceleration.divisor; });

	map.template setWriteHandler<FactorObjects::AccelerationFactorDivisor>(+[](uint32_t value) {
		state_.scalingFactors_.acceleration.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<FactorObjects::Polarity>(
		+[]() { return state_.scalingFactors_.getPolarity(); });

	map.template setWriteHandler<FactorObjects::Polarity>(+[](uint8_t value) {
		state_.scalingFactors_.setPolarity(value);
		return SdoErrorCode::NoError;
	});

	(Modes::template registerHandlers<ObjectDictionary, state_>(map), ...);
}
