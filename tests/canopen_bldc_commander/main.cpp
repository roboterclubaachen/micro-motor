#include <iostream>
#include <modm-canopen/canopen_master.hpp>
#include <modm-canopen/sdo_client.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <thread>
#include <cassert>
#include <array>

#include <modm-canopen/cia402/state_machine.hpp>
#include <modm/platform/can/socketcan.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>

#include <micro-motor/test/csv_writer.hpp>

using modm_canopen::Address;
using modm_canopen::CanopenMaster;
using modm_canopen::SdoErrorCode;
using modm_canopen::cia402::OperatingMode;
using modm_canopen::generated::DefaultObjects;

using namespace std::literals;

int16_t commandedVoltage = 6000;
int16_t outputVoltage = 0;
int32_t velocityValue = 0;
int32_t positionValue = 0;
int32_t errorValue = 0;
OperatingMode currMode = OperatingMode::Voltage;
OperatingMode receivedMode = OperatingMode::Disabled;

constexpr char canDevice[] = "vcan0";

modm::PeriodicTimer debugTimer{100ms};
modm_canopen::cia402::CommandWord control_{0};
modm_canopen::cia402::StateMachine state_{modm_canopen::cia402::State::SwitchOnDisabled};

struct Test
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
	{
		map.template setReadHandler<Address{0x2001, 0}>(+[]() { return uint8_t(10); });

		map.template setReadHandler<Address{0x2002, 0}>(+[]() { return commandedVoltage; });

		map.template setWriteHandler<Address{0x2002, 0}>(
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });
		map.template setReadHandler<Address{0x2003, 0}>(+[]() { return (int16_t)0; });

		map.template setWriteHandler<Address{0x2003, 0}>(+[](int16_t value) {
			if (outputVoltage != value)
			{
				MODM_LOG_INFO << "Received Output Voltage of " << value << modm::endl;
				outputVoltage = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Address{0x6041, 0}>(+[]() { return (uint16_t)0; });

		map.template setWriteHandler<Address{0x6041, 0}>(+[](uint16_t value) {
			state_.set(value);
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Address{0x6040, 0}>(+[]() { return control_.value(); });

		map.template setWriteHandler<Address{0x6040, 0}>(
			+[](uint16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setReadHandler<Address{0x6060, 0}>(+[]() { return (int8_t)currMode; });

		map.template setWriteHandler<Address{0x6060, 0}>(
			+[](int8_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setReadHandler<Address{0x6061, 0}>(+[]() { return (int8_t)receivedMode; });

		map.template setWriteHandler<Address{0x6061, 0}>(+[](int8_t value) {
			if ((int8_t)receivedMode != value)
			{
				MODM_LOG_INFO << "Received Mode " << value << modm::endl;
				receivedMode = (OperatingMode)value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Address{0x6064, 0}>(+[]() { return (int32_t)0; });

		map.template setWriteHandler<Address{0x6064, 0}>(+[](int32_t value) {
			if (positionValue != value)
			{
				// MODM_LOG_INFO << "Received Output Position of " << value << modm::endl;
				positionValue = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Address{0x2004, 0}>(+[]() { return (int32_t)0; });

		map.template setWriteHandler<Address{0x2004, 0}>(+[](int32_t value) {
			if (errorValue != value) { errorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Address{0x606C, 0}>(+[]() { return (int32_t)0; });

		map.template setWriteHandler<Address{0x606C, 0}>(+[](int32_t value) {
			if (velocityValue != value)
			{
				// MODM_LOG_INFO << "Received Output Velocity of " << value << modm::endl;
				velocityValue = value;
			}
			return SdoErrorCode::NoError;
		});
	}
};

struct CommandSendInfo
{
	modm_canopen::cia402::StateCommandNames name;
	OperatingMode mode;
	size_t time;
};

constexpr std::array sendCommands{
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::Shutdown},
					.mode{OperatingMode::Voltage},
					.time{10}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::SwitchOn},
					.mode{OperatingMode::Voltage},
					.time{20}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Velocity},
					.time{30}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::DisableVoltage},
					.mode{OperatingMode::Voltage},
					.time{4000}},
};

int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Position", "Velocity", "Voltage", "Error"}};
	if (!writer.create("vel.csv"))
	{
		MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
		return 1;
	}

	uint64_t counter = 0;
	using Device = CanopenMaster<DefaultObjects, Test>;
	using SdoClient = modm_canopen::SdoClient<Device>;
	const uint8_t motorId = 1;  // Keep consistent with firmware

	modm::platform::SocketCan can;
	bool success = can.open(canDevice);

	if (!success)
	{
		MODM_LOG_ERROR << "Opening device " << canDevice << " failed" << modm::endl;
		return 1;
	}
	MODM_LOG_INFO << "Opened device " << canDevice << modm::endl;

	auto sendMessage = [&can](const modm::can::Message& msg) { return can.sendMessage(msg); };
	auto handleResponse = [&success](const modm_canopen::Address address,
									 const modm_canopen::SdoErrorCode err) {
		MODM_LOG_INFO << "Got response for 0x" << modm::hex << address.index << modm::ascii << "."
					  << address.subindex << ": " << modm::hex << (uint32_t)err << modm::endl;
		if (err != modm_canopen::SdoErrorCode::NoError) { success = false; }
	};

	Device::ReceivePdo_t statusRpdoMotor{};
	statusRpdoMotor.setInactive();
	assert(statusRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Address{0x6041, 0}, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Address{0x2003, 0}, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(2, modm_canopen::PdoMapping{Address{0x6061, 0}, 8}) ==
		   SdoErrorCode::NoError);
	statusRpdoMotor.setMappingCount(3);
	assert(statusRpdoMotor.setActive() == SdoErrorCode::NoError);
	Device::setRPDO(motorId, 0, statusRpdoMotor);
	Device::configureRemoteTPDO(motorId, 0, statusRpdoMotor, sendMessage);

	Device::ReceivePdo_t infoRpdoMotor{};
	infoRpdoMotor.setInactive();
	assert(infoRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Address{0x606C, 0}, 32}) ==
		   SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Address{0x6064, 0}, 32}) ==
		   SdoErrorCode::NoError);
	infoRpdoMotor.setMappingCount(2);
	assert(infoRpdoMotor.setActive() == SdoErrorCode::NoError);
	Device::setRPDO(motorId, 1, infoRpdoMotor);
	Device::configureRemoteTPDO(motorId, 1, infoRpdoMotor, sendMessage);

	Device::ReceivePdo_t errorRpdoMotor{};
	errorRpdoMotor.setInactive();
	assert(errorRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Address{0x2004, 0}, 32}) ==
		   SdoErrorCode::NoError);
	errorRpdoMotor.setMappingCount(1);
	assert(errorRpdoMotor.setActive() == SdoErrorCode::NoError);
	Device::setRPDO(motorId, 2, errorRpdoMotor);
	Device::configureRemoteTPDO(motorId, 2, errorRpdoMotor, sendMessage);

	Device::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{Address{0x6040, 0}, 16}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{Address{0x2002, 0}, 16}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(2, modm_canopen::PdoMapping{Address{0x6060, 0}, 8}) ==
		   SdoErrorCode::NoError);
	commandTpdoMotor.setMappingCount(3);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Device::setTPDO(motorId, 0, commandTpdoMotor);
	Device::configureRemoteRPDO(motorId, 0, commandTpdoMotor, sendMessage);
	SdoClient::requestWrite(motorId, Address{0x60FF, 0}, (uint32_t)10, sendMessage);

	MODM_LOG_INFO << "Waiting on configuration of remote device..." << modm::endl;
	while (SdoClient::waiting())
	{
		if (can.isMessageAvailable())
		{
			modm::can::Message message{};
			can.getMessage(message);
			Device::processMessage(message, handleResponse);
		}
		Device::update(sendMessage);
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
	}
	if (!success)
	{
		MODM_LOG_ERROR << "Configuration failed!" << modm::endl;
		return 1;
	}
	MODM_LOG_INFO << "Configuration done." << modm::endl;
	while (true)
	{
		if (can.isMessageAvailable())
		{
			modm::can::Message message{};
			can.getMessage(message);
			Device::processMessage(message, handleResponse);
		}
		Device::update(sendMessage);
		for (auto c : sendCommands)
		{
			if (c.time == counter)
			{
				MODM_LOG_DEBUG << "Next Command..." << modm::endl;
				control_.apply(modm_canopen::cia402::StateCommands[(uint8_t)c.name].cmd);
				currMode = c.mode;
				Device::setValueChanged(Address{0x6040, 0});
				Device::setValueChanged(Address{0x6060, 0});
			}
		}
		if (debugTimer.execute())
		{
			writer.addRow(
				{std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
				 std::to_string(positionValue % 6), std::to_string(velocityValue),
				 std::to_string((float)outputVoltage / std::numeric_limits<int16_t>::max()),
				 std::to_string(errorValue)});
			writer.flush();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
		counter++;
	}
}
