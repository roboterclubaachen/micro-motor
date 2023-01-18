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

#include <modm/debug/logger.hpp>
#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/platform/can/socketcan.hpp>

#include <canopen/canopen.hpp>
#include <canopen/motor.hpp>

#include <test/csv_writer.hpp>

using namespace std::literals;

modm::PeriodicTimer debugTimer{100ms};

modm::platform::SocketCan can;
constexpr char canDevice[] = "vcan0";
int
main()
{
	auto start = modm::Clock::now();
	CSVWriter writer{{"Time", "Position", "Velocity", "Acceleration"}};
	if (!writer.create("vel.csv"))
	{
		MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
		return 1;
	}

	constexpr uint8_t nodeId = 1;
	const bool success = can.open(canDevice);

	if (!success)
	{
		MODM_LOG_ERROR << "Opening device " << canDevice << " failed" << modm::endl;
		return 1;
	}
	MODM_LOG_INFO << "Opened device " << canDevice << modm::endl;
	auto sendMessage = +[](const modm::can::Message& msg) { return can.sendMessage(msg); };
	CanOpen::initialize(nodeId);

	Motor0.initializeHall();

	while (1)
	{

		while (can.isMessageAvailable())
		{
			modm::can::Message message;
			can.getMessage(message);
			CanOpen::processMessage(message, sendMessage);
		}

		if (Motor0.update()) { CanOpen::setControllerUpdated(); }
		CanOpen::update(sendMessage);

		if (debugTimer.execute())
		{
			writer.addRow({std::to_string((float)(modm::Clock::now() - start).count() / 1000.0f),
						   std::to_string(Motor0.dummy().position()),
						   std::to_string(Motor0.dummy().velocity()),
						   std::to_string(Motor0.dummy().acceleration())});
			writer.flush();
			MODM_LOG_DEBUG << "dummy A: " << Motor0.dummy().acceleration() << modm::endl;
			MODM_LOG_DEBUG << "dummy V: " << Motor0.dummy().velocity() << modm::endl;
			MODM_LOG_DEBUG << "dummy P: " << Motor0.dummy().position() << modm::endl;
			MODM_LOG_DEBUG << "position: " << Motor0.position() << "\n" << modm::endl;
		}
	}
	writer.close();
	return 0;
}
