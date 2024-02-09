/* main.hpp
 *
 * Copyright (C) 2022-2024 Michael Jossen
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
#include <micro-motor/canopen/canopen.hpp>

#include <cstdlib>
#include <vector>

#include "motor.hpp"
#include <gnuplot/gnuplot-iostream.h>

using namespace std::literals;

/*int
main()
{
	Gnuplot gp;
	std::vector<double> t, a, b, c;

	for (double time = 0.0; time < M_PI * 2; time += M_PI / 1000.0)
	{
		auto emf = sim::MotorSimulation::emfFunction(time);
		t.push_back(time);
		a.push_back(emf[0]);
		b.push_back(emf[1]);
		c.push_back(emf[2]);
	}

	auto file = gp.file1d(std::tuple{t, a, b, c});
	gp << "set xrange [-1:7]\nset yrange [-2:2]\n";
	gp << "plot " << file << " using 1:2 with linespoints linetype 6 linewidth 3 title 'A', \\\n\
	 " << file
	   << " using 1:3 with linespoints linetype 7 linewidth 3 title 'B', \\\n\
	 " << file
	   << " using 1:4 with linespoints linetype 2 linewidth 3 title 'C'\n"
	   << std::endl;

	return 0;
}*/

modm::PeriodicTimer debugTimer{1000ms};

modm::platform::SocketCan can;
constexpr char canDevice[] = "vcan0";
int
main(int argc, char** argv)
{

	uint8_t nodeId = 10;
	if (argc == 2) { nodeId = std::atoi(argv[1]); }
	MODM_LOG_INFO << "NodeId: " << nodeId << modm::endl;

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

		Motor0.update(sendMessage);
		CanOpen::update(sendMessage);

		if (debugTimer.execute())
		{
			MODM_LOG_DEBUG << nodeId << " position: " << MotorControl0::state().actualPosition_
						   << modm::endl;
		}
	}
	return 0;
}
