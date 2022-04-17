#pragma once


#include <stdint.h>
#include <modm/io/iostream.hpp>

#include <array>
#include <variant>

namespace motorCan
{

class Configuration
{
public:
	// A board is a microcontroller two motors (or less)
	static constexpr uint8_t
	BoardCount =  5;

	// All motors are enumerated from 0 to MotorCount - 1.
	static constexpr uint8_t
	MotorCount = BoardCount * 2;

public:
	// ID of the sync packet
	static constexpr uint16_t
	sync_id = 0x0000;

	// Length of the sync packet
	static constexpr uint8_t
	sync_length = 0;

	// ID of the first motor
	static constexpr uint16_t
	base_id = 0x10;

	// ID of the first motor reply packets
	static constexpr uint16_t
	base_id_reply = 0x80;
};

template<typename Fdcan>
void setupCanFilters(uint8_t boardId)
{
	Fdcan::setStandardFilter(0, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::sync_id),
		modm::can::StandardMask(0x7ff));

	Fdcan::setStandardFilter(1, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::base_id + boardId),
		modm::can::StandardMask(0x7ff));
}

class DataToMotor
{
public:
	void updateFromMessageData(const uint8_t* data)
	{
		pwmM1 = (data[0] << 8) | data[1];
		pwmM2 = (data[2] << 8) | data[3];
		currentLimitM1 = (data[4] << 8) | data[5];
		currentLimitM2 = (data[6] << 8) | data[7];
	}
public:
	// Pulse-width ratio of 10 bit PWM register
	// -1023 .. -1: negative PWM
	//           0: no PWM
	// +1 .. +1023: positive PWM
	int16_t
	pwmM1;
	int16_t
	pwmM2;

	// Current limit that is limited by the motor hardware
	uint16_t
	currentLimitM1;
	uint16_t
	currentLimitM2;
};

class DataFromMotor
{
public:
	void toMessageData(uint8_t* a) const
	{
		a[0] = encoderCounterRawM1 >> 8;
		a[1] = encoderCounterRawM1 & 0xff;
		a[2] = encoderCounterRawM2 >> 8;
		a[3] = encoderCounterRawM2 & 0xff;
		a[4] = currentM1 >> 8;
		a[5] = currentM1 & 0xff;
		a[6] = currentM2 >> 8;
		a[7] = currentM2 & 0xff;
	}
public:
	uint16_t encoderCounterRawM1;
	uint16_t encoderCounterRawM2;

	// Actual combined motor current in milliamperes
	// Negative current means regenerative breaking
	int16_t currentM1;
	int16_t currentM2;
};

struct Sync{};

using CanMessage = std::variant<std::monostate, Sync, DataToMotor>;

template<typename Fdcan>
CanMessage getCanMessage(uint8_t boardId)
{
	while(Fdcan::isMessageAvailable())
	{
		modm::can::Message canMessage;
		if (!Fdcan::getMessage(canMessage)) {
			break;
		}

		if (canMessage.getIdentifier() == uint32_t(Configuration::base_id + boardId) && canMessage.getLength() == 8) {
			DataToMotor data;
			data.updateFromMessageData(canMessage.data);
			return data;
		} else if (canMessage.getIdentifier() == Configuration::sync_id) {
			return Sync{};
		}
	}

	return std::monostate{};
}

template<typename Fdcan>
void sendResponse(const DataFromMotor& data, uint8_t boardId)
{
	modm::can::Message canMessage(Configuration::base_id_reply + boardId, 8);
	canMessage.setExtended(false);
	data.toMessageData(canMessage.data);
	Fdcan::sendMessage(canMessage);
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

}
