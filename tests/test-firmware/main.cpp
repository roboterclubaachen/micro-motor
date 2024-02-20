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

namespace micro_motor
{
MODM_ISR(TIM1_UP_TIM16)
{
	micro_motor::updateADC();
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
	Motor0.updateMotor();
}
}  // namespace micro_motor

bool
test_zero_current()
{

	MODM_LOG_INFO << "Measuring current at no load... (2s)" << modm::endl;

	bool first = true;
	std::array<float, 3> minCurrent{}, maxCurrent{}, avgCurrentAcc{};
	uint32_t avgCurrentNum{};

	auto testStart = modm::Clock::now();
	while (modm::Clock::now() - testStart < 2s)
	{
		std::array<float, 3> currents = micro_motor::getADCCurrents();
		for (size_t i = 0; i < 3; i++)
		{
			if (currents[i] < minCurrent[i] || first) { minCurrent[i] = currents[i]; }
			if (currents[i] > maxCurrent[i] || first) { maxCurrent[i] = currents[i]; }
			avgCurrentAcc[i] += currents[i];
		}
		avgCurrentNum++;
		first = false;
		modm::delay_us(100);
	}
	constexpr float epsilon = 0.1f;

	avgCurrentAcc[0] /= avgCurrentNum;
	avgCurrentAcc[1] /= avgCurrentNum;
	avgCurrentAcc[2] /= avgCurrentNum;

	MODM_LOG_INFO << "Zero Current test done!" << modm::endl;
	MODM_LOG_INFO << "Currents: [min,avg,max]" << modm::endl;

	MODM_LOG_INFO << "Phase U: [" << minCurrent[0] << "," << (avgCurrentAcc[0]) << ","
				  << maxCurrent[0] << "]" << modm::endl;

	MODM_LOG_INFO << "Phase V: [" << minCurrent[1] << "," << (avgCurrentAcc[1]) << ","
				  << maxCurrent[1] << "]" << modm::endl;

	MODM_LOG_INFO << "Phase W: [" << minCurrent[2] << "," << (avgCurrentAcc[2]) << ","
				  << maxCurrent[2] << "]" << modm::endl;

	bool success = true;
	for (size_t i = 0; i < 3; i++)
	{
		char phase = (i == 0 ? 'U' : (i == 2 ? 'V' : 'W'));
		if (minCurrent[i] < -epsilon)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " minimum current is bigger than allowed!"
						  << modm::endl;
			MODM_LOG_INFO << minCurrent[i] << " < " << -epsilon << modm::endl;
		}
		if (maxCurrent[i] > epsilon)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " maximum current is bigger than allowed!"
						  << modm::endl;
			MODM_LOG_INFO << maxCurrent[i] << " > " << epsilon << modm::endl;
		}
		if (avgCurrentAcc[i] < -epsilon / 2.0f || avgCurrentAcc[i] > epsilon / 2.0f)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " average current is outside of allowed window!"
						  << modm::endl;
			MODM_LOG_INFO << avgCurrentAcc[i] << " < " << -epsilon / 2.0f << " || "
						  << avgCurrentAcc[i] << " > " << epsilon / 2.0f << modm::endl;
		}
	}
	if (!success) { MODM_LOG_INFO << "Failed zero current test!" << modm::endl; }
	return success;
}

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
	MODM_LOG_INFO << "Hardware ID: 0x" << modm::hex << hardwareId << modm::endl;
	MODM_LOG_INFO << "Node ID: " << nodeId << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();
	Board::MotorCurrent::setCurrentLimit(0xFFFF / 8);  // Set current limit to 12.5%
	Motor0.initializeHall();

	size_t failedTests = 0;
	if (!test_zero_current()) { failedTests++; }

	if (failedTests != 0)
	{
		MODM_LOG_ERROR << "Failed " << failedTests << " Tests!" << modm::endl;
	} else
	{
		MODM_LOG_INFO << "All Tests passed!" << modm::endl;
	}
	while (1) {}

	return 0;
}
