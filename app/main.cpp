/* main.hpp
 *
 * Copyright (C) 2018-2021 Raphael Lehmann
 * Copyright (C) 2021 Christopher Durand
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
#include <modm-canopen/canopen_device.hpp>
#include <modm/debug/logger.hpp>
#include <modm/driver/motor/drv832x_spi.hpp>

#include <micro-motor/hardware.hpp>

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);


using modm_canopen::Address;
using modm_canopen::CanopenDevice;
using modm_canopen::SdoErrorCode;
using modm_canopen::generated::DefaultObjects;

uint32_t value2002 = 42;

struct CanOpenHandlers
{
	template<typename ObjectDictionary>
	constexpr void registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
	{
		map.template setReadHandler<Address{0x2001, 0}>(
			+[](){ return uint8_t(10); });

		map.template setReadHandler<Address{0x2002, 0}>(
			+[](){ return value2002; });

		map.template setWriteHandler<Address{0x2002, 0}>(
			+[](uint32_t value)
			{
				MODM_LOG_INFO << "setting 0x2002,0 to " << value << modm::endl;
				value2002 = value;
				return SdoErrorCode::NoError;
			});
	}
};

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();

	Board::Ui::LedRed::set();
	Board::Ui::LedGreen::reset();

	using Can = Board::CanBus::Can;

	const modm::can::Message m{};
	Can::sendMessage(m);

	Can::setStandardFilter(0, Can::FilterConfig::Fifo0, modm::can::StandardIdentifier{}, modm::can::StandardMask{});

	using Device = CanopenDevice<DefaultObjects, CanOpenHandlers>;
	const uint8_t nodeId = 5;
	Device::initialize(nodeId);

	auto sendMessage = [](const modm::can::Message& message) {
		Can::sendMessage(message);
		Board::Ui::LedRed::toggle();
	};

	Device::setValueChanged(Address{0x2002, 0});

	while (1)
	{
		if (Can::isMessageAvailable())
		{
			modm::can::Message message{};
			Can::getMessage(message);
			Device::processMessage(message, sendMessage);

			Board::Ui::LedGreen::toggle();
		}
		Device::update(sendMessage);
	}

	return 0;
}
