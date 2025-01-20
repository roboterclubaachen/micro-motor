#include <modm/platform/can/socketcan.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include "motor_state.hpp"
#include "canopen_callbacks.hpp"
#include "csv_writer.hpp"
#include "relay.hpp"
#include "motor_config.hpp"
#include "relay_analyzer.hpp"
#include "relay_analyzer_simple.hpp"

#include <thread>
#include <cassert>
#include <array>
#include <iostream>

using namespace std::literals;

modm::platform::SocketCan can;
constexpr char canDevice[] = "vcan0";

auto sendMessage = [](const modm::can::Message& msg) { return can.sendMessage(msg); };

RelayAnalyzerSimple::AnalysisMode currentMode = RelayAnalyzerSimple::AnalysisMode::Current;
Relay relay = Relay(RelayConfig{});

int
main()
{
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

	configure(sendMessage);

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
		auto now = modm::Clock::now();
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

		if (!makeReady())
		{
			Master::update(sendMessage);
			continue;
		}

		relay.setValues(state_.currentValue, state_.velocityValue, state_.positionValue);
		relay.update(now);
		if (relay.errored())
		{
			MODM_LOG_ERROR << "Relay errored out, aborting..." << modm::endl;
			break;
		}
		if (relay.done())
		{
			MODM_LOG_INFO << "Relay done!" << modm::endl;
			break;
		}
		if (state_.targetCurrent != relay.getDemand())
		{
			state_.targetCurrent = relay.getDemand();
			motorNode_.setValueChanged(CurrentObjects::TargetCurrent);
		}

		Master::update(sendMessage);

		std::this_thread::sleep_for(std::chrono::milliseconds{1});
	}
	if (!relay.errored())
	{
		relay.dumpToCSV();
		RelayAnalyzerSimple analyzer{RelayAnalyzerSimple::AnalysisMode::Current};
		analyzer.setData(relay.getData());
		if (analyzer.calc())
		{
			auto res = analyzer.getResult().value();
			MODM_LOG_INFO << "Computed PID values:" << modm::endl;
			MODM_LOG_INFO << "P: " << res.p << modm::endl;
			MODM_LOG_INFO << "I: " << res.i << modm::endl;
			MODM_LOG_INFO << "D: " << res.d << modm::endl;
			if (currentMode == RelayAnalyzerSimple::AnalysisMode::Current)
			{
				state_.cPID_kP = res.p;
				state_.cPID_kI = res.i;
				state_.cPID_kD = res.d;
			}
			updatePIDs(sendMessage);
		}
		analyzer.dumpToCSV();
		return 0;
	}

	while (true) {}  // Spin

	return 0;
}
