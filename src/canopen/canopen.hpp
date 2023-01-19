#pragma once
#include <cstdint>
#include <limits>
#include <modm-canopen/canopen_device.hpp>
#include <modm/debug/logger.hpp>

#include "canopen_handlers.hpp"

class CanOpen
{
public:
	using Device =
		modm_canopen::CanopenDevice<modm_canopen::generated::DefaultObjects, CanOpenHandlers>;

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
CanOpen::setControllerUpdated()
{
	Device::setValueChanged(Objects::StatusWord);
	Device::setValueChanged(Objects::OutputPWM);
	Device::setValueChanged(Objects::ModeOfOperation);
	Device::setValueChanged(Objects::VelocityActualValue);
	Device::setValueChanged(Objects::PositionActualValue);
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
