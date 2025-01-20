/*
 * Copyright (C) 2018 Raphael Lehmann <raphael@rleh.de>
 * Copyright (C) 2018 Sebastian Birke <git@se-bi.de>
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
#include <modm/debug/logger/logger.hpp>

namespace {
	using DebugUart = Board::Communication::DebugUart::Uart;
	modm::IODeviceWrapper<DebugUart, modm::IOBuffer::BlockIfFull> loggerDevice;
}

modm::log::Logger modm::log::debug{loggerDevice};
modm::log::Logger modm::log::info{loggerDevice};
modm::log::Logger modm::log::warning{loggerDevice};
modm::log::Logger modm::log::error{loggerDevice};
