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

#include <xpcc/architecture/platform.hpp>

#include "../hardware_rev1.hpp"

int
main()
{
	Board::initializeMcu();
	Board::Ui::initialize();

	Board::Ui::LedRed::set();
	Board::Ui::LedBlue::set();

	while (1)
	{
		Board::Ui::LedRed::toggle();
		xpcc::delayMilliseconds(Board::Ui::PadC14::read() ? 250 : 500);
	}

	return 0;
}
