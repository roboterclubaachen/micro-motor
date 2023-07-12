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
constexpr uint8_t motorId = 22;  // Keep consistent with firmware

modm::PeriodicTimer debugTimer{10ms};
modm_canopen::cia402::CommandWord control_{0};
modm_canopen::cia402::StateMachine state_{modm_canopen::cia402::State::SwitchOnDisabled};

#ifdef HOSTED
constexpr char canDevice[] = "vcan0";
#else
constexpr char canDevice[] = "can0";
#endif

constexpr float vPID_kP = 0.00014f;
constexpr float vPID_kI = 0.000003f;
constexpr float vPID_kD = 0.0f;
int32_t targetSpeed = 128;
int32_t velDemand = 0;

constexpr float pPID_kP = 2.0f;
constexpr float pPID_kI = 0.0f;
constexpr float pPID_kD = 0.0f;
int32_t targetPosition = 0;
int32_t posDemand = 0;

constexpr float cPID_kP = -50000.0f;
constexpr float cPID_kI = -800.0f;
constexpr float cPID_kD = -0.1f;
float targetCurrent = 0.0f;
float commandedCurrent = 0.0f;
float maxCharge = 0.0f;
float currentCharge = 0.0f;

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

		map.template setWriteHandler<Objects::PositionDemandValue, int32_t>(+[](int32_t value) {
			if (posDemand != value) { posDemand = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityDemandValue, int32_t>(+[](int32_t value) {
			if (velDemand != value) { velDemand = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::VelocityError, int32_t>(+[](int32_t value) {
			if (velErrorValue != value) { velErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::FilteredActualCurrent, float>(+[](float value) {
			if (currentValue != value) { currentValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::CommandedCurrent, float>(+[](float value) {
			if (commandedCurrent != value) { commandedCurrent = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::CurrentError, float>(+[](float value) {
			if (currentErrorValue != value) { currentErrorValue = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::MaxCharge, float>(+[](float value) {
			if (maxCharge != value) { maxCharge = value; }
			return SdoErrorCode::NoError;
		});

		map.template setWriteHandler<Objects::CurrentCharge, float>(+[](float value) {
			if (currentCharge != value) { currentCharge = value; }
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
	assert(statusRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::StatusWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::OutputPWM, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(2, modm_canopen::PdoMapping{Objects::ModeOfOperationDisplay,
																  8}) == SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMappingCount(3) == SdoErrorCode::NoError);
	assert(statusRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 0, statusRpdoMotor);
	Master::configureRemoteTPDO(motorId, 0, statusRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t infoRpdoMotor{};
	infoRpdoMotor.setInactive();
	assert(infoRpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::FilteredActualCurrent,
																32}) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::CommandedCurrent, 32}) ==
		   SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 1, infoRpdoMotor);
	Master::configureRemoteTPDO(motorId, 1, infoRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t test2RpdoMotor{};
	test2RpdoMotor.setInactive();
	assert(test2RpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::VelocityActualValue,
																 32}) == SdoErrorCode::NoError);
	assert(test2RpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::PositionActualValue,
																 32}) == SdoErrorCode::NoError);
	assert(test2RpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(test2RpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 2, test2RpdoMotor);
	Master::configureRemoteTPDO(motorId, 2, test2RpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t test3RpdoMotor{};
	test3RpdoMotor.setInactive();
	assert(test3RpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::VelocityDemandValue,
																 32}) == SdoErrorCode::NoError);
	assert(test3RpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::PositionDemandValue,
																 32}) == SdoErrorCode::NoError);
	assert(test3RpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(test3RpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 3, test3RpdoMotor);
	Master::configureRemoteTPDO(motorId, 3, test3RpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{Objects::ControlWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{Objects::ModeOfOperation, 8}) ==
		   SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 0, commandTpdoMotor);
	Master::configureRemoteRPDO(motorId, 0, commandTpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, Objects::TargetCurrent, targetCurrent,
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

	SdoClient::requestWrite(motorId, Objects::CurrentPID_kP, cPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::CurrentPID_kI, cPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, Objects::CurrentPID_kD, cPID_kD,
							std::forward<MessageCallback>(sendMessage));
}

size_t maxTime = 5100;

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
						SdoClient::requestWrite(motorId, Objects::TargetPosition, targetSpeed,
												sendMessage);
						control_.setBit<modm_canopen::cia402::CommandBits::NewSetPoint>(true);
					}}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::DisableVoltage},
					.mode{OperatingMode::Voltage},
					.time{5030},
					.custom{nullptr}},
};

int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Current", "Commanded", "Velocity", "VelocityTarget", "Position",
					  "PositionTarget", "PWM", "Mode"}};
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
				motorNode_.setValueChanged(Objects::ControlWord);
				motorNode_.setValueChanged(Objects::ModeOfOperation);
			}
		}
		if (debugTimer.execute())
		{
			writer.addRow({std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
						   std::to_string(currentValue), std::to_string(commandedCurrent),
						   std::to_string(velocityValue), std::to_string(velDemand),
						   std::to_string(positionValue), std::to_string(posDemand),
						   std::to_string(outputPWM), std::to_string(receivedMode)});
			writer.flush();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
		counter++;
		if (counter == maxTime) { break; }
	}
	return 0;
}
