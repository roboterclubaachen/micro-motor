#pragma once
#include <cstdint>
#include <limits>
#include <modm-canopen/generated/micro-motor_od.hpp>

#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/debug/logger.hpp>

#include <librobots2/motor-canopen/current_protocol.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <librobots2/motor-canopen/pwm_protocol.hpp>
#include <librobots2/motor-canopen/error_protocol.hpp>
#include <librobots2/motor-canopen/velocity_protocol.hpp>
#include <librobots2/motor-canopen/position_protocol.hpp>

template<size_t id>
using MotorControl_t = MotorControl<id, IdentityProtocol<id>, HeartbeatProtocol<id>,
									PWMProtocol<id>, VelocityProtocol<id>, PositionProtocol<id>,
									QuickstopProtocol<id>, CurrentProtocol<id>>;

using MotorControl0 = MotorControl_t<0>;

class CanOpen
{
public:
	using Device =
		modm_canopen::CanopenDevice<modm_canopen::generated::micromotor_OD, MotorControl0>;

	static inline void
	initialize(uint8_t nodeId);
	static inline void
	setControllerUpdated();
	static inline void
	processMessage(const modm::can::Message &message,
				   bool (*sendMessage)(const modm::can::Message &));
	static inline void
	update(bool (*sendMessage)(const modm::can::Message &));
};

void
CanOpen::initialize(uint8_t nodeId)
{
	Device::initialize(nodeId);
}

void
CanOpen::processMessage(const modm::can::Message &message,
						bool (*sendMessage)(const modm::can::Message &))
{
	Device::processMessage(message, sendMessage);
}

void
CanOpen::update(bool (*sendMessage)(const modm::can::Message &))
{
	Device::update(sendMessage);
}
