/* main.hpp
 *
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

#include <modm/platform/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include "../hardware_rev1.hpp"

#include <modm/driver/motor/drv832x_spi.hpp>


modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm::PeriodicTimer aliveTimer{100};
modm::PeriodicTimer gateDriverStatusTimer{1000};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::LedRed::reset();
	Board::Ui::LedBlue::set();
	MODM_LOG_ERROR << "Micro-Motor Gatedriver Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();

	Board::Motor::setCompareValue(Board::Motor::MaxPwm / 2);
	Board::Motor::MotorTimer::applyAndReset();
	Board::Motor::MotorTimer::start();
	Board::Motor::MotorTimer::enableOutput();

	while (1)
	{
		Board::Ui::LedRed::set(Board::MotorBridge::GateDriverFault::read());
		modm::delayMilliseconds(1);
		if(aliveTimer.execute()) {
			Board::Ui::LedBlue::set();
			modm::delayMilliseconds(1);
			Board::Ui::LedBlue::reset();
		}
		if(gateDriverStatusTimer.execute()) {

			RF_CALL_BLOCKING(gateDriver.readAll());

			MODM_LOG_DEBUG << gateDriver.faultStatus1() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.vgsStatus2() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.driverControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveHS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveLS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.ocpControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.csaControl() << modm::endl;
			MODM_LOG_ERROR << modm::endl;

			gateDriver.gateDriveHS() &= ~modm::drv832xSpi::HS_IDriveN_t::mask();
			gateDriver.gateDriveHS() |= modm::drv832xSpi::HS_IDriveN_t(modm::drv832xSpi::IDriveN::mA740);
		}
	}

	return 0;
}
