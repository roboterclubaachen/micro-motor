#ifndef HEARTBEAT_PROTOCOL_HPP
#error "Do not include this file directly, use heartbeat_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>
template <size_t id>
void HeartbeatProtocol<id>::makeHeartbeatMSG(uint8_t canId,
                                             modm::can::Message &message) {
  message = modm::can::Message{0x700 + (uint32_t)canId, 1};
  message.setExtended(false);
  message.data[0] = 0x05; // Always report operational, as we otherwise would
                          // not be able to send
}
template <size_t id>
template <typename Device, typename MessageCallback>
bool HeartbeatProtocol<id>::update(MotorState &, MessageCallback &&cb) {
  if (heartBeatTimer_.execute()) {
    modm::can::Message message;
    makeHeartbeatMSG(Device::nodeId(), message);
    cb(message);
  }
  return true;
}
template <size_t id>
template <typename ObjectDictionary, const MotorState &state>
constexpr void HeartbeatProtocol<id>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {
  using modm_canopen::SdoErrorCode;
  map.template setReadHandler<HeartbeatObjects::TimeBetweenHeartbeats>(
      +[]() { return (uint16_t)timeBetweenHeatbeats.count(); });

  map.template setWriteHandler<HeartbeatObjects::TimeBetweenHeartbeats>(
      +[](uint16_t value) {
        timeBetweenHeatbeats = std::chrono::milliseconds((int64_t)value);
        heartBeatTimer_ = modm::PeriodicTimer{timeBetweenHeatbeats / 2};
        return SdoErrorCode::NoError;
      });
}