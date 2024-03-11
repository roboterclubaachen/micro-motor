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
#include <librobots2/motor-sim/current_limit.hpp>

#include <cstdlib>
#include <vector>
#include <limits>

#include "motor.hpp"
#include "csv_writer.hpp"

using namespace std::literals;
#define LOG
#define CAN

#ifdef LOG
modm::PeriodicTimer logTimer{1000us};
#endif

#ifdef CAN

modm::PeriodicTimer debugTimer{4000ms};

modm::platform::SocketCan can;
constexpr char canDevice[] = "vcan0";
int
main(int argc, char** argv)
{

#ifdef LOG
	CSVWriter writer{{"Time", "v1", "v2", "v3", "i1", "i2", "i3", "theta", "omega", "e1", "e2",
					  "e3", "pwm", "te", "tl", "tf", "g1", "g2", "g3"}};
	if (!writer.create("sim_motor.csv"))
	{
		MODM_LOG_ERROR << "Failed to create CSV File!" << modm::endl;
		return 1;
	}
#endif

	uint8_t nodeId = 12;
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

	librobots2::motor_sim::MotorData simConfig{
		.l{0.515},
		.j{0.0001},
		.f_l{0.0005},
		.f_s{0.001},
	};
	librobots2::motor_sim::MotorSimulation::initialize(simConfig);

	Motor0.initializeHall();
	auto start = modm::Clock::now();

	while (1)
	{
		auto now = modm::Clock::now();
		auto diff = now - start;

		Motor0.update(sendMessage);
		while (can.isMessageAvailable())
		{
			modm::can::Message message;
			can.getMessage(message);
			CanOpen::processMessage(message, sendMessage);
			Motor0.update(sendMessage);
		}
#ifdef LOG
		auto& state = librobots2::motor_sim::MotorSimulation::state();
		auto& gates = librobots2::motor_sim::MotorBridge::getConfig();
		if (logTimer.execute())
		{
			writer.addRowC(diff.count(), state.v[0], state.v[1], state.v[2], state.i[0], state.i[1],
						   state.i[2], state.theta_m, state.omega_m, state.e[0], state.e[1],
						   state.e[2], MotorState0::outputPWM() * 3, state.t_e, state.t_l,
						   state.t_f, (int)gates[0] - 1, (int)gates[1] - 1, (int)gates[2] - 1);
		}
#endif

		Motor0.update(sendMessage);
		CanOpen::update(sendMessage);
	}
	return 0;
}

#endif

#ifdef SIMPLE

int
main()
{

	MODM_LOG_INFO << "Running test..." << modm::endl;
	CSVWriter writer{{"Time", "v1", "v2", "v3", "i1", "i2", "i3", "theta", "omega", "e1", "e2",
					  "e3", "pwm", "te", "tl", "tf", "g1", "g2", "g3"}};
	if (!writer.create("sim_motor.csv"))
	{
		MODM_LOG_ERROR << "Failed to create CSV File!" << modm::endl;
		return 1;
	}

	librobots2::motor_sim::MotorData simConfig{};
	librobots2::motor_sim::MotorSimulation::initialize(simConfig);

	Motor0.initializeHall();
	auto start = modm::Clock::now();
	while (1)
	{
		auto now = modm::Clock::now();
		auto diff = now - start;
		// int pwm = ((diff.count() / 400) % 2) * std::numeric_limits<int16_t>::max();
		//  int pwm = std::sin(diff.count() / 1000.0f) * 32000.0f;
		int pwm = std::numeric_limits<int16_t>::max();
		double currentLimit = std::abs(std::sin(diff.count() / 1000.0f) * 2.0);
		librobots2::motor_sim::CurrentLimit::set(currentLimit);
		Motor0.testUpdate(pwm);
#ifdef LOG
		if (logTimer.execute())
		{
			auto& state = librobots2::motor_sim::MotorSimulation::state();
			auto& pwms = librobots2::motor_sim::MotorBridge::getPWMs();
			auto& gates = librobots2::motor_sim::MotorBridge::getConfig();
			writer.addRowC(diff.count(), state.v[0], state.v[1], state.v[2], state.i[0], state.i[1],
						   state.i[2], state.theta_m, state.omega_m, state.e[0], state.e[1],
						   state.e[2], pwms[0] * 3, state.t_e, state.t_l, state.t_f,
						   (int)gates[0] - 1, (int)gates[1] - 1, (int)gates[2] - 1);
			writer.flush();
		}
#endif
		if (diff > 10s) return 0;
	}
	return 0;
}

#endif