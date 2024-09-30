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

#include <micro-motor/micro-motor.hpp>
#include <modm/driver/motor/drv832x_spi.hpp>

#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>

#include <micro-motor/canopen/canopen.hpp>
#include "motor.hpp"

#include <info_build.h>
#include <info_git.h>

inline modm::IODeviceWrapper<Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull> loggerDevice;
inline modm::log::Logger modm::log::debug(loggerDevice);
inline modm::log::Logger modm::log::info(loggerDevice);
inline modm::log::Logger modm::log::warning(loggerDevice);
inline modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::PeriodicTimer debugTimer{500ms};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs>
	gateDriver;

namespace
{

struct BoardId
{
	uint32_t id;
	uint32_t nodeId;
	uint8_t hw_version_major;
	uint8_t hw_version_minor;
};
uint8_t
readBoardId()
{
	static constexpr std::array boards = {
		// hardware id, board id
		BoardId{.id = 0x00320025u, .nodeId = 1u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x0031002cu, .nodeId = 2u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x00340045u, .nodeId = 3u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x00340047u, .nodeId = 4u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002c0048u, .nodeId = 5u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002b0041u, .nodeId = 6u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002b0045u, .nodeId = 7u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002e002cu, .nodeId = 8u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002e002du, .nodeId = 9u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x00320041u, .nodeId = 10u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x002c004du, .nodeId = 11u, .hw_version_major = 2, .hw_version_minor = 1},
		BoardId{.id = 0x00390046u, .nodeId = 12u, .hw_version_major = 2, .hw_version_minor = 2},
		BoardId{.id = 0x00290044, .nodeId = 13u, .hw_version_major = 2, .hw_version_minor = 2},
		BoardId{.id = 0x002E0046, .nodeId = 14u, .hw_version_major = 2, .hw_version_minor = 2},
		BoardId{.id = 0x00410046, .nodeId = 15u, .hw_version_major = 2, .hw_version_minor = 2},
	};

	const auto hardwareId = Board::readHardwareId();
	auto it = std::find_if(std::begin(boards), std::end(boards),
						   [hardwareId](auto board) { return board.id == hardwareId; });
	if (it == std::end(boards))
	{
		MODM_LOG_INFO << "Hardware ID: 0x" << modm::hex << hardwareId << modm::endl;
		MODM_LOG_ERROR << "Board not found" << modm::endl;
		Board::Ui::LedRed::set();
		while (1) asm volatile("nop");
	}
	if (it->hw_version_major != Board::Version_Major ||
		it->hw_version_minor != Board::Version_Minor)
	{
		MODM_LOG_INFO << "Hardware ID: 0x" << modm::hex << hardwareId << modm::endl;
		MODM_LOG_ERROR << "Wrong hardware version!" << modm::endl;
		MODM_LOG_ERROR << "Is v" << Board::Version_Major << "." << Board::Version_Minor
					   << modm::endl;
		MODM_LOG_ERROR << "Should be v" << it->hw_version_major << "." << it->hw_version_minor
					   << modm::endl;
		Board::Ui::LedRed::set();
		while (1) asm volatile("nop");
	}

	return it->nodeId;
}

}  // namespace

constexpr auto noMessageTimeout = 200ms;

int
main()
{

	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();
	micro_motor::initialize();

	MODM_LOG_ERROR << "Micro-Motor Application controlling BLDC Motors via canOpen interace"
				   << modm::endl;
	MODM_LOG_INFO << "Compiled for board version v" << Board::Version_Major << "."
				  << Board::Version_Minor << modm::endl;

	// Let's print some information about the compiling host, user etc.
	MODM_LOG_INFO << "Machine:  " << MODM_BUILD_MACHINE << modm::endl;
	MODM_LOG_INFO << "User:     " << MODM_BUILD_USER << modm::endl;
	MODM_LOG_INFO << "Os:       " << MODM_BUILD_OS << modm::endl;
	MODM_LOG_INFO << "Compiler: " << MODM_BUILD_COMPILER << modm::endl;
	// Let's print some information about the git statud, that is provided in the
	// modm_git_info.hpp
	MODM_LOG_INFO << "Local Git User:" << modm::endl;
	MODM_LOG_INFO << "Name:  " << MODM_GIT_CONFIG_USER_NAME << "<" << MODM_GIT_CONFIG_USER_EMAIL
				  << ">" << modm::endl;
	MODM_LOG_INFO << "Last Commit:" << modm::endl;
	MODM_LOG_INFO << "Abbreviated SHA: " << MODM_GIT_SHA_ABBR << modm::endl;
	MODM_LOG_INFO << "Subject:         " << MODM_GIT_SUBJECT << modm::endl;
	MODM_LOG_INFO << modm::endl << "Author:" << modm::endl;
	MODM_LOG_INFO << "Name:      " << MODM_GIT_AUTHOR << modm::endl;
	MODM_LOG_INFO << "Email:     " << MODM_GIT_AUTHOR_EMAIL << modm::endl;
	MODM_LOG_INFO << "Date:      " << MODM_GIT_AUTHOR_DATE << modm::endl;
	MODM_LOG_INFO << "Timestamp: " << MODM_GIT_AUTHOR_DATE_TIMESTAMP << modm::endl;
	MODM_LOG_INFO << modm::endl << "File Status:" << modm::endl;
	MODM_LOG_INFO << "Mod " << MODM_GIT_MODIFIED << " Add " << MODM_GIT_ADDED << " Del "
				  << MODM_GIT_DELETED;
	MODM_LOG_INFO << " Ren " << MODM_GIT_RENAMED << " Cop " << MODM_GIT_COPIED << " Unt "
				  << MODM_GIT_UNTRACKED << modm::endl;
	MODM_LOG_INFO << modm::endl;

	const uint8_t boardId = readBoardId();
	const uint8_t nodeId = 2 * boardId;
	MODM_LOG_INFO.printf("Board ID: %d\n", boardId);
	MODM_LOG_INFO.printf("Canopen Node ID: %d\n", nodeId);

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();
	Board::MotorCurrent::setCurrentLimit(0xFFFF);  // Set current limit to 100%

	Motor0.initializeHall();

	Board::CanBus::Can::setStandardFilter(0, Board::CanBus::Can::FilterConfig::Fifo0,
										  modm::can::StandardIdentifier(0),
										  modm::can::StandardMask(0));

	Board::CanBus::Can::setExtendedFilter(0, Board::CanBus::Can::FilterConfig::Fifo0,
										  modm::can::ExtendedIdentifier(0),
										  modm::can::ExtendedMask(0));

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
		/*if ((now - lastMessage) >= noMessageTimeout)
		{
			MODM_LOG_ERROR << "No messages! Resetting...\n" << modm::endl;
			break;
		}
		if (MotorState0::resetMotor_)
		{
			MODM_LOG_ERROR << "Resetting...\n" << modm::endl;
			break;
		}*/

		if (debugTimer.execute())
		{
			/*MODM_LOG_DEBUG << "MotorState:\n"
						   << "Max Current: " << MotorState0::maxCurrent_ << "\n"
						   << "Commanded Current: " << CurrentControl<0>::commandedCurrent_ << "\n"
						   << "Target Current: " << CurrentProtocol<0>::targetCurrent_ << "\n"
						   << "Unoriented Current: " << MotorState0::unorientedCurrent_ << "\n"
						   << "Oriented Current: " << MotorState0::orientedCurrent_ << "\n"
						   << "Commanded Velocity: " << VelocityProtocol<0>::commandedVelocity_
						   << "\n"
						   << "Actual Velocity: " << MotorState0::actualVelocity_.getValue() << "\n"
						   << "Commanded Position: " << PositionProtocol<0>::commandedPosition_
						   << "\n"
						   << "Actual Position: " << MotorState0::actualPosition_ << "\n"
						   << "Mode: " << MotorState0::mode_ << "\n"
						   << "Charge: " << MotorState0::currentCharge_ << "\n"
						   << "PWM: " << MotorState0::outputPWM_ << modm::endl;*/
		}
	}

	return 0;
}
