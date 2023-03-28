#ifndef PWM_PROTOCOL_HPP
#error "Do not include this file directly, use pwm_protocol.hpp instead"
#endif
#include <modm/debug/logger.hpp>

template<typename Device, typename MessageCallback>
bool
PWMProtocol::update(MotorState& state, MessageCallback&&)
{
	state.outputPWM_ = commandedPWM_;
	state.status_.setBit<modm_canopen::cia402::StatusBits::TargetReached>(true);
	return true;
}

template<typename ObjectDictionary, const MotorState& state>
constexpr void
PWMProtocol::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<PWMObjects::PWMCommand>(+[]() { return commandedPWM_; });

	map.template setWriteHandler<PWMObjects::PWMCommand>(+[](int16_t value) {
		commandedPWM_ = value;
		MODM_LOG_INFO << "Set commanded PWM to " << commandedPWM_ << modm::endl;
		return SdoErrorCode::NoError;
	});
}
