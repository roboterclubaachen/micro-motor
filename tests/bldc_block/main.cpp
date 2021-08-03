/* main.hpp
*
* Copyright (C) 2021 Christopher Durand
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

#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <micro-motor/hardware.hpp>

#include <modm/driver/motor/drv832x_spi.hpp>

#include <librobots2/motor/bldc_motor_block_commutation.hpp>

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::PeriodicTimer debugTimer{100ms};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	MODM_LOG_ERROR << "Micro-Motor BLDC Motor block commutation Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();

	using librobots2::motor::BldcMotorBlockCommutation;
	BldcMotorBlockCommutation<Board::Motor> motor{5};

	motor.setSetpoint(7000);

	while (1)
	{
		if (debugTimer.execute()) {
			MODM_LOG_DEBUG << "hall: " << Board::Motor::HallPort::read() << modm::endl;
		}
		motor.update();
	}

	return 0;
}
