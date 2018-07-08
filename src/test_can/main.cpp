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

#include "../hardware_rev1.hpp"

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
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::LedRed::set();
	Board::Ui::LedBlue::reset();
	MODM_LOG_ERROR << "Micro-Motor CAN Test" << modm::endl;

	MODM_LOG_INFO << "Setting up Filter for Can ..." << modm::endl;
	// Receive every message
	CanFilter::setFilter(0, CanFilter::FIFO0,
			CanFilter::ExtendedIdentifier(0),
			CanFilter::ExtendedFilterMask(0));

	// Send a message
	MODM_LOG_INFO << "Sending message on Can ..." << modm::endl;
	modm::can::Message msg1(1, 1);
	msg1.setExtended(true);
	msg1.data[0] = 0x11;
	Board::CanBus::Can::sendMessage(msg1);

	modm::PeriodicTimer pTimer(100);

	while (1)
	{
		if (Board::CanBus::Can::isMessageAvailable())
		{
			MODM_LOG_INFO << "Can: Message is available..." << modm::endl;
			modm::can::Message message;
			Board::CanBus::Can::getMessage(message);
			displayMessage(message);
		}

		if (pTimer.execute()) {
			Board::Ui::LedBlue::reset();

			static uint8_t idx = 0;
			modm::can::Message msg1(1, 1);
			msg1.setExtended(true);
			msg1.data[0] = idx;
			Board::CanBus::Can::sendMessage(msg1);

			++idx;
		}
	}


	return 0;
}
