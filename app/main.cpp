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
#include <modm/debug/logger.hpp>
#include <modm/driver/motor/drv832x_spi.hpp>

#include <micro-motor/hardware.hpp>
#include <librobots2/motor/dc_motor.hpp>

#include "motor_can.hpp"

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

using DcMotor = librobots2::motor::DcMotor<Board::Motor>;

namespace
{

void initialize()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();

	Board::Ui::LedRed::reset();
	Board::Ui::LedGreen::set();

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::setCompareValue(0);
	Board::Motor::MotorTimer::start();
	Board::Motor::configure(Board::Motor::PhaseConfig::Low);
}

void syncReceived(uint8_t boardId)
{
	Board::Ui::LedGreen::toggle();
	motorCan::DataFromMotor data{};
	data.encoderCounterRawM1 = Board::Encoder::getEncoderRaw();
	// TODO: current measurement
	data.currentM1 = Board::MotorCurrent::AdcV::getValue() - 0x7ff;
	motorCan::sendResponse<Board::CanBus::Can>(data, boardId);
}

void processCommand(const motorCan::DataToMotor& command, DcMotor& motor)
{
	motor.setSetpoint(command.pwmM1);
	Board::MotorCurrent::setCurrentLimit(command.currentLimitM1);
}

uint32_t readHardwareId()
{
	return *((volatile uint32_t *) UID_BASE);
}

uint8_t readBoardId()
{
	static constexpr std::array boards = {
		// hardware id, board id
		std::pair{0x0032003au, 1u}
	};

	const auto hardwareId = readHardwareId();
	auto it = std::find_if(std::begin(boards), std::end(boards), [hardwareId](auto board) {
		return board.first == hardwareId;
	});
	if (it == std::end(boards)) {
		MODM_LOG_ERROR << "Board not found" << modm::endl;
		while(1) asm volatile("nop");
	}

	return it->second;
}

}

int main()
{
	initialize();

	MODM_LOG_INFO << "Micro-Motor DC-Motor hack" << modm::endl;
	MODM_LOG_INFO.printf("Hardware ID: 0x%08lx\n", readHardwareId());

	const uint8_t boardId = readBoardId();
	MODM_LOG_INFO.printf("Board ID: %d\n", boardId);

	motorCan::setupCanFilters<Board::CanBus::Can>(boardId);
	DcMotor motor;

	while (1)
	{
		using namespace motorCan;

		std::visit(overloaded {
			[](std::monostate) {}, // No message
			[&](const Sync&) { syncReceived(boardId); },
			[&](const DataToMotor& command) { processCommand(command, motor); }
		},
		motorCan::getCanMessage<Board::CanBus::Can>(boardId));
	}

	return 0;
}
