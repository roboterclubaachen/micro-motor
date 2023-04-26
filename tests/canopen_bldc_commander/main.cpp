#include <modm/platform/can/socketcan.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

#include <micro-motor/canopen/canopen_objects.hpp>

#include <modm-canopen/canopen_master.hpp>
#include <modm-canopen/sdo_client.hpp>

#include <thread>
#include <cassert>
#include <array>
#include <iostream>

#include "csv_writer.hpp"

using modm_canopen::Address;
using modm_canopen::CanopenMaster;
using modm_canopen::CanopenNode;
using modm_canopen::SdoErrorCode;
using modm_canopen::cia402::OperatingMode;
using modm_canopen::generated::DefaultObjects;

using namespace std::literals;

struct CommandSendInfo
{
	modm_canopen::cia402::StateCommandNames name;
	OperatingMode mode;
	size_t time;
	void (*custom)(){nullptr};
};
uint32_t updateTime = 0;
bool targetReached = true;
int16_t commandedPWM = 6000;
int16_t outputPWM = 0;
int32_t velocityValue = 0;
int32_t positionValue = 0;
int32_t velErrorValue = 0;
int32_t posErrorValue = 0;
OperatingMode currMode = OperatingMode::Voltage;
OperatingMode receivedMode = OperatingMode::Disabled;

modm::platform::SocketCan can;
constexpr uint8_t motorId = 1;  // Keep consistent with firmware

modm::PeriodicTimer debugTimer{10ms};
modm_canopen::cia402::CommandWord control_{0};
modm_canopen::cia402::StateMachine state_{modm_canopen::cia402::State::SwitchOnDisabled};
#define HOSTED
#ifdef HOSTED
constexpr char canDevice[] = "vcan0";
constexpr float vPID_kP = 150.0f;
constexpr float vPID_kI = 22.0f;
constexpr float vPID_kD = 200.5f;
int32_t targetSpeed = 10;

constexpr float pPID_kP = 0.05f;
constexpr float pPID_kI = 0.05f;
constexpr float pPID_kD = 0.05f;
int32_t targetPosition = 0;
#else
// TODO tune
constexpr char canDevice[] = "can0";
constexpr float vPID_kP = 210.0f;
constexpr float vPID_kI = 62.0f;
constexpr float vPID_kD = 200.0f;
int32_t targetSpeed = 10;

constexpr float pPID_kP = 1.0f;
constexpr float pPID_kI = 0.01f;
constexpr float pPID_kD = 0.01f;
int32_t targetPosition = 0;
#endif

struct Test
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(uint8_t, modm_canopen::HandlerMapRT<ObjectDictionary>& map)
	{
		map.template setWriteHandler<Objects::UpdateTime, uint32_t>(+[](uint32_t value) {
			updateTime = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::PWMCommand, int16_t>(+[]() { return commandedPWM; });

		map.template setWriteHandler<Objects::PWMCommand, int16_t>(
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setWriteHandler<Objects::OutputPWM, int16_t>(+[](int16_t value) {
			if (outputPWM != value)
			{
				// MODM_LOG_INFO << "Received Output PWM of " << value << modm::endl;
				outputPWM = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::StatusWord, uint16_t>(+[](uint16_t value) {
			state_.set(value);
			if (state_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() && !targetReached)
			{
				MODM_LOG_INFO << "Target Reached!" << modm::endl;
				targetReached = true;
			}
			if (!state_.isSet<modm_canopen::cia402::StatusBits::TargetReached>() && targetReached)
			{
				MODM_LOG_INFO << "Target Lost!" << modm::endl;
				targetReached = false;
			}
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<Objects::ControlWord, uint16_t>(
			+[]() { return control_.value(); });

		map.template setReadHandler<Objects::ModeOfOperation, int8_t>(
			+[]() { return (int8_t)currMode; });

		map.template setWriteHandler<Objects::ModeOfOperationDisplay, int8_t>(+[](int8_t value) {
			if ((int8_t)receivedMode != value)
			{
				MODM_LOG_INFO << "Received Mode " << value << modm::endl;
				receivedMode = (OperatingMode)value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::PositionActualValue, int32_t>(+[](int32_t value) {
			if (positionValue != value)
			{
				// MODM_LOG_INFO << "Received Output Position of " << value << modm::endl;
				positionValue = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityError, int32_t>(+[](int32_t value) {
			if (velErrorValue != value) { velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::FollowingErrorActualValue, int32_t>(
			+[](int32_t value) {
				if (posErrorValue != value) { posErrorValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<Objects::VelocityActualValue, int32_t>(+[](int32_t value) {
			if (velocityValue != value)
			{
				// MODM_LOG_INFO << "Received Output Velocity of " << value << modm::endl;
				velocityValue = value;
			}
			return SdoErrorCode::NoError;
		});
	}
};

using MotorNode = CanopenNode<DefaultObjects, Test>;
using Master = CanopenMaster<MotorNode>;
using SdoClient = modm_canopen::SdoClient<Master>;

auto sendMessage = [](const modm::can::Message& msg) { return can.sendMessage(msg); };

template<typename MessageCallback>
void
setPDOs(MessageCallback&& sendMessage)
{
	MotorNode::ReceivePdo_t statusRpdoMotor{};
	statusRpdoMotor.setInactive();
	assert(statusRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::StatusWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::OutputPWM, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(2, modm_canopen::PdoMapping{Objects::ModeOfOperationDisplay,
																  8}) == SdoErrorCode::NoError);
	statusRpdoMotor.setMappingCount(3);
	assert(statusRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 0, statusRpdoMotor);
	Master::configureRemoteTPDO(motorId, 0, statusRpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t infoRpdoMotor{};
	infoRpdoMotor.setInactive();
	assert(infoRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::VelocityActualValue,
																32}) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::PositionActualValue,
																32}) == SdoErrorCode::NoError);
	infoRpdoMotor.setMappingCount(2);
	assert(infoRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 1, infoRpdoMotor);
	Master::configureRemoteTPDO(motorId, 1, infoRpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t errorRpdoMotor{};
	errorRpdoMotor.setInactive();
	assert(errorRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::VelocityError, 32}) ==
		   SdoErrorCode::NoError);
	assert(errorRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::FollowingErrorActualValue,
																 32}) == SdoErrorCode::NoError);
	errorRpdoMotor.setMappingCount(2);
	assert(errorRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 2, errorRpdoMotor);
	Master::configureRemoteTPDO(motorId, 2, errorRpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t testRpdoMotor{};
	testRpdoMotor.setInactive();
	assert(testRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::UpdateTime, 32}) ==
		   SdoErrorCode::NoError);
	testRpdoMotor.setMappingCount(1);
	assert(testRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 3, testRpdoMotor);
	Master::configureRemoteTPDO(motorId, 3, testRpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::ControlWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::PWMCommand, 16}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(2, modm_canopen::PdoMapping{Objects::ModeOfOperation, 8}) ==
		   SdoErrorCode::NoError);
	commandTpdoMotor.setMappingCount(3);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 0, commandTpdoMotor);
	Master::configureRemoteRPDO(motorId, 0, commandTpdoMotor,
								std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::TargetVelocity, targetSpeed,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::TargetPosition, targetPosition,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, Objects::VelocityPID_kP, vPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::VelocityPID_kI, vPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::VelocityPID_kD, vPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, Objects::PositionPID_kP, pPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::PositionPID_kI, pPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::PositionPID_kD, pPID_kD,
							std::forward<MessageCallback>(sendMessage));
}

size_t maxTime = 15000;

constexpr std::array sendCommands{
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::Shutdown},
					.mode{OperatingMode::Voltage},
					.time{10},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::SwitchOn},
					.mode{OperatingMode::Voltage},
					.time{20},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Velocity},
					.time{30},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Velocity},
					.time{1030},
					.custom{[]() {
						targetSpeed = -targetSpeed;
						SdoClient::requestWrite(motorId, Objects::TargetVelocity, targetSpeed,
												sendMessage);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Position},
					.time{2030},
					.custom{[]() {
						SdoClient::requestWrite(motorId, Objects::TargetPosition, targetSpeed,
												sendMessage);
						control_.setBit<modm_canopen::cia402::CommandBits::NewSetPoint>(true);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Position},
					.time{7030},
					.custom{[]() {
						targetPosition = 500;
						SdoClient::requestWrite(motorId, Objects::TargetPosition, targetPosition,
												sendMessage);
						control_.setBit<modm_canopen::cia402::CommandBits::NewSetPoint>(true);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Position},
					.time{13030},
					.custom{[]() {
						SdoClient::requestWrite(motorId, Objects::PositionWindow, (uint32_t)400,
												sendMessage);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::DisableVoltage},
					.mode{OperatingMode::Voltage},
					.time{14030},
					.custom{nullptr}},
};

int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Position", "Velocity", "PWM", "VelError", "PosError", "Time"}};
	if (!writer.create("vel.csv"))
	{
		MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
		return 1;
	}

	uint64_t counter = 0;

	bool success = can.open(canDevice);

	if (!success)
	{
		MODM_LOG_ERROR << "Opening device " << canDevice << " failed" << modm::endl;
		return 1;
	}
	MODM_LOG_INFO << "Opened device " << canDevice << modm::endl;

	auto& motorNode_ = Master::addDevice<MotorNode>(motorId);

	auto handleResponse = [&success](const uint8_t id, const modm_canopen::Address address,
									 const modm_canopen::SdoErrorCode err) {
		MODM_LOG_INFO << "Got response for node " << id << " 0x" << modm::hex << address.index
					  << modm::ascii << "." << address.subindex << ": " << modm::hex
					  << (uint32_t)err << modm::endl;
		if (err != modm_canopen::SdoErrorCode::NoError) { success = false; }
	};

	setPDOs(sendMessage);

	MODM_LOG_INFO << "Waiting on configuration of remote device..." << modm::endl;
	while (SdoClient::waiting())
	{
		if (can.isMessageAvailable())
		{
			modm::can::Message message{};
			can.getMessage(message);
			Master::processMessage(message, handleResponse);
		}
		Master::update(sendMessage);
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
		while (SdoClient::waiting())
		{
			if (can.isMessageAvailable())
			{
				modm::can::Message message{};
				can.getMessage(message);
				Master::processMessage(message, handleResponse);
			}
			Master::update(sendMessage);
			std::this_thread::sleep_for(std::chrono::milliseconds{1});
		}

		if (can.isMessageAvailable())
		{
			modm::can::Message message{};
			can.getMessage(message);
			Master::processMessage(message, handleResponse);
		}
		Master::update(sendMessage);
		for (auto c : sendCommands)
		{
			if (c.time == counter)
			{
				MODM_LOG_DEBUG << "Next Command..." << modm::endl;
				control_.apply(modm_canopen::cia402::StateCommands[(uint8_t)c.name].cmd);
				currMode = c.mode;
				if (c.custom != nullptr) c.custom();
				motorNode_.setValueChanged(Objects::ControlWord);
				motorNode_.setValueChanged(Objects::ModeOfOperation);
			}
		}
		if (debugTimer.execute())
		{
			writer.addRow({std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
						   std::to_string(positionValue), std::to_string(velocityValue),
						   std::to_string((float)outputPWM / std::numeric_limits<int16_t>::max()),
						   std::to_string(velErrorValue), std::to_string(posErrorValue),
						   std::to_string(updateTime)});
			writer.flush();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
		counter++;
		if (counter == maxTime) { break; }
	}
	return 0;
}
