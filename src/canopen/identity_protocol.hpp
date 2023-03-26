#ifndef IDENTITY_PROTOCOL_HPP
#define IDENTITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "motor_state.hpp"

#include <chrono>
#include <modm/processing/timer.hpp>

struct IdentityObjects
{
	static constexpr modm_canopen::Address DeviceType{0x1000, 0};
	static constexpr modm_canopen::Address ErrorRegister{0x1001, 0};
	static constexpr modm_canopen::Address IdentityObject{0x1018, 0};
	static constexpr modm_canopen::Address VendorId{0x1018, 1};
	static constexpr modm_canopen::Address ProductCode{0x1018, 2};
	static constexpr modm_canopen::Address RevisionId{0x1018, 3};
	static constexpr modm_canopen::Address SerialNumber{0x1018, 4};
};

class IdentityProtocol
{
public:
	static bool
	applicable(const MotorState& state)
	{
		return false;
	}

	template<typename Device, typename MessageCallback>
	static bool
	update(MotorState&, MessageCallback&&)
	{}

	template<typename ObjectDictionary, const MotorState& state>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);
};

#include "identity_protocol_impl.hpp"
#endif