/* main.hpp
 *
 * Copyright (C) 2022-2023 Michael Jossen
 * Copyright (C) 2021-2022 Christopher Durand
 * Copyright (C) 2018 Raphael Lehmann <raphael@rleh.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <micro-motor/hardware.hpp>
#include <modm/driver/motor/drv832x_spi.hpp>

#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>

#include <micro-motor/canopen/canopen.hpp>
#include "motor.hpp"

inline modm::IODeviceWrapper<Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull> loggerDevice;
inline modm::log::Logger modm::log::debug(loggerDevice);
inline modm::log::Logger modm::log::info(loggerDevice);
inline modm::log::Logger modm::log::warning(loggerDevice);
inline modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::PeriodicTimer debugTimer{100ms};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs>
	gateDriver;

namespace
{

uint8_t
readBoardId()
{
	static constexpr std::array boards = {
		// hardware id, board id
		std::pair{0x00320025u, 1u},  std::pair{0x0031002cu, 2u}, std::pair{0x00340045u, 3u},
		std::pair{0x00340047u, 4u},  std::pair{0x002c0048u, 5u}, std::pair{0x002b0041u, 6u},
		std::pair{0x002b0045u, 7u},  std::pair{0x002e002cu, 8u}, std::pair{0x002e002du, 9u},
		std::pair{0x00320041u, 10u}, std::pair{0x002c004du, 11u}};

	const auto hardwareId = Board::readHardwareId();
	auto it = std::find_if(std::begin(boards), std::end(boards),
						   [hardwareId](auto board) { return board.first == hardwareId; });
	if (it == std::end(boards))
	{
		MODM_LOG_INFO << "Hardware ID: " << modm::hex << hardwareId << modm::endl;
		MODM_LOG_ERROR << "Board not found" << modm::endl;
		while (1) asm volatile("nop");
	}

	return it->second;
}

}  // namespace

constexpr auto noMessageTimeout = 200ms;

int
main()
{

	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();
	Board::MotorCurrent::setCurrentLimit(0xFFFF / 4);  // Set current limit to 25%

	MODM_LOG_ERROR << "Micro-Motor Application controlling BLDC Motors via canOpen interace"
				   << modm::endl;

	const uint8_t boardId = readBoardId();
	MODM_LOG_INFO.printf("Board ID: %d\n", boardId);

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();

	Motor0.initializeHall();

	Board::CanBus::Can::setStandardFilter(0, Board::CanBus::Can::FilterConfig::Fifo0,
										  modm::can::StandardIdentifier(0),
										  modm::can::StandardMask(0));

	Board::CanBus::Can::setExtendedFilter(0, Board::CanBus::Can::FilterConfig::Fifo0,
										  modm::can::ExtendedIdentifier(0),
										  modm::can::ExtendedMask(0));

	const uint8_t nodeId = 2 * boardId;
	MODM_LOG_INFO.printf("Node ID: %d\n", nodeId);
	CanOpen::initialize(nodeId);
	auto lastMessage = modm::Clock::now();
	MODM_LOG_INFO << "Starting App Main Loop..." << modm::endl;
	while (1)
	{
		auto now = modm::Clock::now();
		using Can = Board::CanBus::Can;
		while (Can::isMessageAvailable())
		{
			modm::can::Message message;
			Can::getMessage(message);
			CanOpen::processMessage(message, Board::CanBus::Can::sendMessage);
			lastMessage = now;
			// MODM_LOG_INFO << modm::hex << message.getIdentifier() << modm::endl;
		}

		Motor0.update(Board::CanBus::Can::sendMessage);
		CanOpen::update(Board::CanBus::Can::sendMessage);
		if ((now - lastMessage) >= noMessageTimeout)
		{
			MODM_LOG_ERROR << "No messages! Resetting...\n" << modm::endl;
			break;
		}
		if (MotorControl0::state().resetMotor_)
		{
			MODM_LOG_ERROR << "Resetting...\n" << modm::endl;
			break;
		}

		auto& state = MotorControl0::state();
		if (debugTimer.execute())
		{
			MODM_LOG_DEBUG
				<< "MotorState:\n"
				<< "Enabled : " << state.enableMotor_ << "\n"
				<< "Position: " << state.actualPosition_ << "\n"
				<< "Target Position: " << PositionProtocol<0>::commandedPosition_ << "\n"
				<< "Velocity: " << state.actualVelocity_.getValue() << "\n"
				<< "Control : " << modm::bin << state.control_.value() << modm::ascii << "\n"
				<< "Mode: " << state.mode_ << "\n"
				<< "PWM: " << state.outputPWM_ << "\n"
				<< "Status: " << modm::bin << state.status_.status() << modm::ascii << "\n"
				<< "Target Reached: "
				<< (state.status_.isSet<modm_canopen::cia402::StatusBits::TargetReached>()
						? "true"
						: "false")
				<< modm::endl;
		}
	}

	return 0;
}
