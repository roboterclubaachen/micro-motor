#define HOSTED
#define PWM_CMDS

#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <thread>
#include <cassert>
#include <array>
#include <iostream>

#include "csv_writer.hpp"
#include "command_send_info.hpp"
#include "handlers.hpp"
#include "canopen.hpp"
#include "setup.hpp"
#include "runs/pwm_run.hpp"

using namespace std::literals;

modm::PeriodicTimer debugTimer{10ms};

int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Current", "Commanded", "Velocity", "VelocityTarget", "Position",
					  "PositionTarget", "PWM", "Mode", "Charge", "MaxCharge", "OrientedCurrent",
					  "OrientedCurrentAngleDiff"}};
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
#if defined(CMDLIST)
		for (auto c : sendCommands)
		{
			if (c.time == counter)
			{
				MODM_LOG_DEBUG << "Next Command..." << modm::endl;
				state.control_.apply(modm_canopen::cia402::StateCommands[(uint8_t)c.name].cmd);
				state.currMode = c.mode;
				if (c.custom != nullptr) c.custom();
				motorNode_.setValueChanged(StateObjects::ControlWord);
				motorNode_.setValueChanged(StateObjects::ModeOfOperation);
			}
		}
#elif defined(SINUS)
		if (10 == counter)
		{
			MODM_LOG_DEBUG << "Next Command..." << modm::endl;
			state.control_.apply(modm_canopen::cia402::StateCommands
									 [(uint8_t)modm_canopen::cia402::StateCommandNames::Shutdown]
										 .cmd);
			state.currMode = OperatingMode::Voltage;
			motorNode_.setValueChanged(StateObjects::ControlWord);
			motorNode_.setValueChanged(StateObjects::ModeOfOperation);
		} else if (20 == counter)
		{
			MODM_LOG_DEBUG << "Next Command..." << modm::endl;
			state.control_.apply(modm_canopen::cia402::StateCommands
									 [(uint8_t)modm_canopen::cia402::StateCommandNames::SwitchOn]
										 .cmd);
			state.currMode = OperatingMode::Voltage;
			motorNode_.setValueChanged(StateObjects::ControlWord);
			motorNode_.setValueChanged(StateObjects::ModeOfOperation);
		} else if (30 == counter)
		{
			MODM_LOG_DEBUG << "Next Command..." << modm::endl;
			state.control_.apply(
				modm_canopen::cia402::StateCommands
					[(uint8_t)modm_canopen::cia402::StateCommandNames::EnableOperation]
						.cmd);
			state.currMode = OperatingMode::Velocity;
			motorNode_.setValueChanged(StateObjects::ControlWord);
			motorNode_.setValueChanged(StateObjects::ModeOfOperation);
		} else if (counter > 100 && counter < 10000 && counter % 100)
		{
			const double offset = counter - 100;
			const auto mult = 600;
			state.targetSpeed = mult * std::sin(offset / 800);
			MODM_LOG_INFO << state.targetSpeed << modm::endl;
			motorNode_.setValueChanged(VelocityObjects::TargetVelocity);
		} else if (counter == 10030)
		{
			MODM_LOG_DEBUG << "Next Command..." << modm::endl;
			state.control_.apply(
				modm_canopen::cia402::StateCommands
					[(uint8_t)modm_canopen::cia402::StateCommandNames::DisableVoltage]
						.cmd);
			state.currMode = OperatingMode::Voltage;
			motorNode_.setValueChanged(StateObjects::ControlWord);
			motorNode_.setValueChanged(StateObjects::ModeOfOperation);
		}
#endif
		if (debugTimer.execute())
		{
			writer.addRow({std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
						   std::to_string(state.currentValue),
						   std::to_string(state.commandedCurrent),
						   std::to_string(state.velocityValue), std::to_string(state.velDemand),
						   std::to_string(state.positionValue), std::to_string(state.posDemand),
						   std::to_string(state.outputPWM), std::to_string(state.receivedMode),
						   std::to_string(state.currentCharge), std::to_string(state.maxCharge),
						   std::to_string(state.orientedCurrent),
						   std::to_string(state.orientedCurrentAngleDiff)});
			writer.flush();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
		counter++;
		if (counter == maxTime) { break; }
	}
	return 0;
}
