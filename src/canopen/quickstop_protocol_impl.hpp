#ifndef QUICKSTOP_PROTOCOL_HPP
#error "Do not include this file directly, use quickstop_protocol.hpp instead"
#endif

template<typename VelocityProtocol>
template<typename Device>
bool
QuickstopProtocol<VelocityProtocol>::update(MotorState& state)
{
	if (state.status_.state() == modm_canopen::cia402::State::QuickStopActive)
	{
		VelocityProtocol::doQuickStopUpdate(quickStopDeceleration_, state);
		Device::setValueChanged(VelocityObjects::VelocityDemandValue);
		Device::setValueChanged(VelocityObjects::VelocityError);
		state.status_.setBit<modm_canopen::cia402::StatusBits::NotCurrentlyQuickStopping>(false);
	} else
	{
		state.status_.setBit<modm_canopen::cia402::StatusBits::NotCurrentlyQuickStopping>(true);
	}
	return true;
}

template<typename VelocityProtocol>
template<typename ObjectDictionary, const MotorState& state>
constexpr void
QuickstopProtocol<VelocityProtocol>::registerHandlers(
	modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<QuickStopObjects::QuickStopDeceleration>(
		+[]() { return state.scalingFactors_.acceleration.toUser(quickStopDeceleration_); });

	map.template setWriteHandler<QuickStopObjects::QuickStopDeceleration>(+[](int32_t value) {
		quickStopDeceleration_ = state.scalingFactors_.acceleration.toInternal(value);
		return SdoErrorCode::NoError;
	});
}