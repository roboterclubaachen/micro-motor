#ifndef QUICKSTOP_PROTOCOL_HPP
#define QUICKSTOP_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "motor_state.hpp"

struct QuickStopObjects
{
	static constexpr modm_canopen::Address QuickStopDeceleration{0x6085, 0};  // User units
};

template<typename VelocityProtocol>
class QuickstopProtocol
{
public:
	static inline int32_t quickStopDeceleration_{10000};

public:
	static bool
	applicable(const MotorState& state)
	{
		return state.enableMotor_;
	}

	template<typename Device, typename MessageCallback>
	static bool
	update(MotorState& state, MessageCallback&& cb);

	template<typename ObjectDictionary, const MotorState& state>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);
};

#include "quickstop_protocol_impl.hpp"
#endif