#pragma once

#include <cstdint>
#include <limits>
#include <micro-motor/hardware.hpp>
#include <modm-canopen/canopen_device.hpp>

#include "canopen_handlers.hpp"

class CanOpen
{
public:
	using Device = modm_canopen::CanopenDevice<modm_canopen::generated::DefaultObjects, CanOpenHandlers>;

	static inline void initialize(uint8_t nodeId);
	static inline void setControllerUpdated();
	static inline void processMessage(const modm::can::Message& message);
	static inline void update();
};

void CanOpen::initialize(uint8_t nodeId)
{
	Device::initialize(nodeId);
}

void CanOpen::setControllerUpdated()
{
	Device::setValueChanged(Objects::PositionActualValue);
	Device::setValueChanged(Objects::VelocityActualValue);
	Device::setValueChanged(Objects::OutputVoltage);
}

void CanOpen::processMessage(const modm::can::Message& message)
{
	Device::processMessage(message, Board::CanBus::Can::sendMessage);
}

void CanOpen::update()
{
	Device::update(Board::CanBus::Can::sendMessage);
}
