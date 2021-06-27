/* main.hpp
 *
 * Copyright (C) 2018 Sebastian Birke <rca@se-bi.de>
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
#include <modm/debug/logger.hpp>
#include <modm/processing/timer.hpp>

#include <micro-motor/hardware.hpp>

// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;

// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

// Set the log level
#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DEBUG

static void
displayMessage(const modm::can::Message& message)
{
	static uint32_t receiveCounter = 0;
	receiveCounter++;

	MODM_LOG_INFO<< "id  =" << message.getIdentifier();
	if (message.isExtended()) {
		MODM_LOG_INFO<< " extended";
	}
	else {
		MODM_LOG_INFO<< " standard";
	}
	if (message.isRemoteTransmitRequest()) {
		MODM_LOG_INFO<< ", rtr";
	}
	MODM_LOG_INFO<< modm::endl;

	MODM_LOG_INFO<< "dlc =" << message.getLength() << modm::endl;
	if (!message.isRemoteTransmitRequest())
	{
		MODM_LOG_INFO << "data=";
		for (uint32_t i = 0; i < message.getLength(); ++i) {
			MODM_LOG_INFO<< modm::hex << message.data[i] << modm::ascii << ' ';
		}
		MODM_LOG_INFO<< modm::endl;
	}
	MODM_LOG_INFO<< "# received=" << receiveCounter << modm::endl;
}

int
main()
{
	using namespace Board::CanBus;

	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::LedRed::set();
	Board::Ui::LedGreen::set();
	MODM_LOG_ERROR << "Micro-Motor CAN Test" << modm::endl;

	MODM_LOG_INFO << "Setting up Filter for Can ..." << modm::endl;
	// Receive every message
	Can::setExtendedFilter(0, Can::FilterConfig::Fifo0,
			modm::can::ExtendedIdentifier(0),
			modm::can::ExtendedMask(0));

	// Send a message
	MODM_LOG_INFO << "Sending message on Can ..." << modm::endl;
	modm::can::Message msg1(1, 1);
	msg1.setExtended(true);
	msg1.data[0] = 0x11;
	Can::sendMessage(msg1);

	modm::PeriodicTimer pTimer(100);

	while (1)
	{
		if (Can::isMessageAvailable())
		{
			MODM_LOG_INFO << "Can: Message is available..." << modm::endl;
			modm::can::Message message;
			Board::CanBus::Can::getMessage(message);
			displayMessage(message);
		}

		if (pTimer.execute()) {
			Board::Ui::LedGreen::toggle();

			static uint8_t idx = 0;
			modm::can::Message msg1(1, 1);
			msg1.setExtended(true);
			msg1.data[0] = idx;
			Can::sendMessage(msg1);

			++idx;
		}
	}

	return 0;
}
