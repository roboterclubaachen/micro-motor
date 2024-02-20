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

#include "motor.hpp"

#include <info_build.h>
#include <info_git.h>

inline modm::IODeviceWrapper<Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull> loggerDevice;
inline modm::log::Logger modm::log::debug(loggerDevice);
inline modm::log::Logger modm::log::info(loggerDevice);
inline modm::log::Logger modm::log::warning(loggerDevice);
inline modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs>
	gateDriver;

int
main()
{

	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();
	micro_motor::initialize();

	MODM_LOG_ERROR << "Micro-Motor Application controlling BLDC Motors via canOpen interace"
				   << modm::endl;

	const auto hardwareId = Board::readHardwareId();
	const uint8_t nodeId = 1;
	MODM_LOG_INFO << "Machine:  " << MODM_BUILD_MACHINE << modm::endl;
	MODM_LOG_INFO << "User:     " << MODM_BUILD_USER << modm::endl;
	MODM_LOG_INFO << "Os:       " << MODM_BUILD_OS << modm::endl;
	MODM_LOG_INFO << "Compiler: " << MODM_BUILD_COMPILER << modm::endl;
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
	MODM_LOG_INFO << "Hardware ID: " << hardwareId << modm::endl;
	MODM_LOG_INFO << "Node ID: " << nodeId << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();
	Board::MotorCurrent::setCurrentLimit(0xFFFF / 8);  // Set current limit to 12.5%

	Motor0.initializeHall();

	MODM_LOG_INFO << "Test done! Looping..." << modm::endl;
	while (1) {}

	return 0;
}
