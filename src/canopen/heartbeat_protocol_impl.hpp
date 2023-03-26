#ifndef HEARTBEAT_PROTOCOL_HPP
#error "Do not include this file directly, use heartbeat_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>

auto
HeartbeatProtocol::makeHeartbeatMSG(uint8_t canId) -> modm::can::Message
{
	modm::can::Message message{0x700 + (uint32_t)canId, 1};
	message.setExtended(false);
	message.data[0] = 0x05;  // Always report operational, as we otherwise would not be able to send
	return message;
}

template<typename Device, typename MessageCallback>
bool
HeartbeatProtocol::update(MotorState&, MessageCallback&& cb)
{
	if (heartBeatTimer_.execute()) { cb(makeHeartbeatMSG(Device::nodeId())); }
	return true;
}

template<typename ObjectDictionary, const MotorState& state>
constexpr void
HeartbeatProtocol::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<HeartbeatObjects::TimeBetweenHeartbeats>(
		+[]() { return (uint16_t)timeBetweenHeatbeats.count(); });

	map.template setWriteHandler<HeartbeatObjects::TimeBetweenHeartbeats>(+[](uint16_t value) {
		timeBetweenHeatbeats = std::chrono::milliseconds((int64_t)value);
		heartBeatTimer_ = modm::PeriodicTimer{timeBetweenHeatbeats};
		return SdoErrorCode::NoError;
	});
}