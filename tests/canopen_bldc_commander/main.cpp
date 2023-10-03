#include <modm/platform/can/socketcan.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

#include <librobots2/motor-canopen/canopen_objects.hpp>

#include <modm-canopen/master/canopen_master.hpp>
#include <modm-canopen/master/sdo_client.hpp>
#include <modm-canopen/generated/micro-motor_od.hpp>

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
using modm_canopen::generated::micromotor_OD;

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
int16_t commandedPWM = 0;
int16_t outputPWM = 0;
int32_t velocityValue = 0;
int32_t positionValue = 0;
float currentValue = 0.0f;
int32_t velErrorValue = 0;
int32_t posErrorValue = 0;
float currentErrorValue = 0.0f;
OperatingMode currMode = OperatingMode::Current;
OperatingMode receivedMode = OperatingMode::Disabled;

modm::platform::SocketCan can;

modm::PeriodicTimer debugTimer{10ms};
modm_canopen::cia402::CommandWord control_{0};
modm_canopen::cia402::StateMachine state_{modm_canopen::cia402::State::SwitchOnDisabled};
// #define HOSTED
#ifdef HOSTED
constexpr char canDevice[] = "vcan0";
constexpr uint8_t motorId = 10;  // Keep consistent with firmware
#else
constexpr char canDevice[] = "can0";
constexpr uint8_t motorId = 22;  // Keep consistent with firmware
#endif

// constexpr float vPID_kP = 0.0002f;
// constexpr float vPID_kI = 0.000005f;
// constexpr float vPID_kD = 0.0000001f;

constexpr float vPID_kP = 0.0002f;
constexpr float vPID_kI = 0.000005f;
constexpr float vPID_kD = 0.0000001f;
int32_t targetSpeed = 2000;
int32_t velDemand = 0;

constexpr float pPID_kP = 2.0f;
constexpr float pPID_kI = 0.0f;
constexpr float pPID_kD = 0.0f;
int32_t targetPosition = 0;
int32_t posDemand = 0;

// constexpr float cPID_kP = -1.0f;
// constexpr float cPID_kI = -0.005f;
// constexpr float cPID_kD = 0.0f;

constexpr float cPID_kP = -1.0f;
constexpr float cPID_kI = -0.005f;
constexpr float cPID_kD = 0.0f;
float targetCurrent = 0.4f;
float commandedCurrent = 0.0f;
float maxCharge = 0.0f;
float currentCharge = 0.0f;

struct Test
{
	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(uint8_t, modm_canopen::HandlerMapRT<ObjectDictionary>& map)
	{
		map.template setWriteHandler<StateObjects::UpdateTime, uint32_t>(+[](uint32_t value) {
			updateTime = value;
			return SdoErrorCode::NoError;
		});

		map.template setReadHandler<PWMObjects::PWMCommand, int16_t>(
			+[]() { return commandedPWM; });

		map.template setWriteHandler<PWMObjects::PWMCommand, int16_t>(
			+[](int16_t) { return SdoErrorCode::UnsupportedAccess; });

		map.template setWriteHandler<StateObjects::OutputPWM, int16_t>(+[](int16_t value) {
			if (outputPWM != value)
			{
				// MODM_LOG_INFO << "Received Output PWM of " << value << modm::endl;
				outputPWM = value;
			}
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::StatusWord, uint16_t>(+[](uint16_t value) {
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

		map.template setReadHandler<StateObjects::ControlWord, uint16_t>(
			+[]() { return control_.value(); });

		map.template setReadHandler<StateObjects::ModeOfOperation, int8_t>(
			+[]() { return (int8_t)currMode; });

		map.template setWriteHandler<StateObjects::ModeOfOperationDisplay, int8_t>(
			+[](int8_t value) {
				if ((int8_t)receivedMode != value)
				{
					MODM_LOG_INFO << "Received Mode " << value << modm::endl;
					receivedMode = (OperatingMode)value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::PositionActualValue, int32_t>(
			+[](int32_t value) {
				if (positionValue != value)
				{
					// MODM_LOG_INFO << "Received Output Position of " << value << modm::endl;
					positionValue = value;
				}
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<PositionObjects::PositionDemandValue, int32_t>(
			+[](int32_t value) {
				if (posDemand != value) { posDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityDemandValue, int32_t>(
			+[](int32_t value) {
				if (velDemand != value) { velDemand = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<VelocityObjects::VelocityError, int32_t>(+[](int32_t value) {
			if (velErrorValue != value) { velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::FilteredActualCurrent, float>(
			+[](float value) {
				if (currentValue != value) { currentValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<CurrentObjects::CommandedCurrent, float>(+[](float value) {
			if (commandedCurrent != value) { commandedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::CurrentError, float>(+[](float value) {
			if (currentErrorValue != value) { currentErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<StateObjects::MaxCharge, float>(+[](float value) {
			if (maxCharge != value) { maxCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<CurrentObjects::CurrentCharge, float>(+[](float value) {
			if (currentCharge != value) { currentCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<PositionObjects::FollowingErrorActualValue, int32_t>(
			+[](int32_t value) {
				if (posErrorValue != value) { posErrorValue = value; }
				return SdoErrorCode::NoError;
			});

		map.template setWriteHandler<StateObjects::VelocityActualValue, int32_t>(
			+[](int32_t value) {
				if (velocityValue != value)
				{
					// MODM_LOG_INFO << "Received Output Velocity of " << value << modm::endl;
					velocityValue = value;
				}
				return SdoErrorCode::NoError;
			});
	}
};

using MotorNode = CanopenNode<micromotor_OD, Test>;
using Master = CanopenMaster<MotorNode>;
using SdoClient = modm_canopen::SdoClient<Master>;

auto sendMessage = [](const modm::can::Message& msg) { return can.sendMessage(msg); };

template<typename MessageCallback>
void
setPDOs(MessageCallback&& sendMessage)
{
	MotorNode::ReceivePdo_t statusRpdoMotor{};
	statusRpdoMotor.setInactive();
	assert(statusRpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::StatusWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(1, modm_canopen::PdoMapping{StateObjects::OutputPWM, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(
			   2, modm_canopen::PdoMapping{StateObjects::ModeOfOperationDisplay, 8}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMappingCount(3) == SdoErrorCode::NoError);
	assert(statusRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 0, statusRpdoMotor);
	Master::configureRemoteTPDO(motorId, 0, statusRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t infoRpdoMotor{};
	infoRpdoMotor.setInactive();
	assert(infoRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{CurrentObjects::FilteredActualCurrent, 32}) ==
		   SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMapping(1, modm_canopen::PdoMapping{CurrentObjects::CommandedCurrent,
																32}) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 1, infoRpdoMotor);
	Master::configureRemoteTPDO(motorId, 1, infoRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t currentRpdoMotor{};
	currentRpdoMotor.setInactive();
	assert(currentRpdoMotor.setMapping(0, modm_canopen::PdoMapping{CurrentObjects::CurrentCharge,
																   32}) == SdoErrorCode::NoError);
	assert(currentRpdoMotor.setMappingCount(1) == SdoErrorCode::NoError);
	assert(currentRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 2, currentRpdoMotor);
	Master::configureRemoteTPDO(motorId, 2, currentRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t velocityRpdoMotor{};
	velocityRpdoMotor.setInactive();
	assert(velocityRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{VelocityObjects::VelocityDemandValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{StateObjects::VelocityActualValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 3, velocityRpdoMotor);
	Master::configureRemoteTPDO(motorId, 3, velocityRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t positionRpdoMotor{};
	positionRpdoMotor.setInactive();
	assert(positionRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{PositionObjects::PositionDemandValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(positionRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{StateObjects::PositionActualValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(positionRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(positionRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 4, positionRpdoMotor);
	Master::configureRemoteTPDO(motorId, 4, positionRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::ControlWord,
																   16}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{StateObjects::ModeOfOperation,
																   8}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 0, commandTpdoMotor);
	Master::configureRemoteRPDO(motorId, 0, commandTpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, CurrentObjects::TargetCurrent, targetCurrent,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::TargetVelocity, targetSpeed,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::TargetPosition, targetPosition,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kP, vPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kI, vPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kD, vPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kP, pPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kI, pPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kD, pPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kP, cPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kI, cPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kD, cPID_kD,
							std::forward<MessageCallback>(sendMessage));
}

size_t maxTime = 10100;

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
					.mode{OperatingMode::Position},
					.time{1030},
					.custom{[]() {
						SdoClient::requestWrite(motorId, PositionObjects::TargetPosition,
												targetPosition, sendMessage);
						control_.setBit<modm_canopen::cia402::CommandBits::NewSetPoint>(true);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Position},
					.time{5030},
					.custom{[]() {
						targetPosition = 500;
						SdoClient::requestWrite(motorId, PositionObjects::TargetPosition,
												targetPosition, sendMessage);
						control_.setBit<modm_canopen::cia402::CommandBits::NewSetPoint>(true);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::DisableVoltage},
					.mode{OperatingMode::Voltage},
					.time{10030},
					.custom{nullptr}},
};

int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Current", "Commanded", "Velocity", "VelocityTarget", "Position",
					  "PositionTarget", "PWM", "Mode", "Charge", "MaxCharge"}};
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
				motorNode_.setValueChanged(StateObjects::ControlWord);
				motorNode_.setValueChanged(StateObjects::ModeOfOperation);
			}
		}
		if (debugTimer.execute())
		{
			writer.addRow({std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
						   std::to_string(currentValue), std::to_string(commandedCurrent),
						   std::to_string(velocityValue), std::to_string(velDemand),
						   std::to_string(positionValue), std::to_string(posDemand),
						   std::to_string(outputPWM), std::to_string(receivedMode),
						   std::to_string(currentCharge), std::to_string(maxCharge)});
			writer.flush();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
		counter++;
		if (counter == maxTime) { break; }
	}
	return 0;
}
