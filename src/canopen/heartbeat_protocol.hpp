#ifndef HEARTBEAT_PROTOCOL_HPP
#define HEARTBEAT_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "motor_state.hpp"

#include <chrono>
#include <modm/processing/timer.hpp>

using namespace std::literals;

struct HeartbeatObjects
{
	static constexpr modm_canopen::Address TimeBetweenHeartbeats{0x1017, 0};
};

class HeartbeatProtocol
{
public:
	static inline auto timeBetweenHeatbeats{100ms};
	static inline modm::PeriodicTimer heartBeatTimer_{timeBetweenHeatbeats};

public:
	static bool
	applicable(const MotorState& state)
	{
		return true;
	}

	template<typename Device, typename MessageCallback>
	static bool
	update(MotorState& state, MessageCallback&& cb);

	template<typename ObjectDictionary, const MotorState& state>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);

private:
	static inline auto
	makeHeartbeatMSG(uint8_t canId) -> modm::can::Message;
};

#include "heartbeat_protocol_impl.hpp"
#endif