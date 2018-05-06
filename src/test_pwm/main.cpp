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

#include "../hardware_rev1.hpp"

modm::PeriodicTimer aliveTimer{100};

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::LedRed::reset();
	Board::Ui::LedBlue::set();

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
	}

	return 0;
}
