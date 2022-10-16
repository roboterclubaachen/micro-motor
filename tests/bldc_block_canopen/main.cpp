/* main.hpp
*
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

#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include <micro-motor/hardware.hpp>

#include <modm/driver/motor/drv832x_spi.hpp>

#include "canopen.hpp"
#include "motor.hpp"

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::PeriodicTimer debugTimer{100ms};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

int main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();

	MODM_LOG_ERROR << "BLDC Motor test with canopen interface" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();

	Motor0.initializeHall();
	Motor0.setEnabled(false);

	constexpr uint8_t nodeId = 1;
	CanOpen::initialize(nodeId);

	while (1)
	{
		if (debugTimer.execute()) {
			MODM_LOG_DEBUG << "hall: " << Board::Motor::HallPort::read() << modm::endl;
			MODM_LOG_DEBUG << "position: " << Motor0.position() << "\n" << modm::endl;
		}

		using Can = Board::CanBus::Can;
		while (Can::isMessageAvailable()) {
			modm::can::Message message;
			Can::getMessage(message);
			CanOpen::processMessage(message);
		}

		if (Motor0.update()) {
			CanOpen::setControllerUpdated();
		}
		CanOpen::update();
	}

	return 0;
}
