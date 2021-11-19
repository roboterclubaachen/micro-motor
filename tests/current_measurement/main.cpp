/* main.hpp
 *
 * Copyright (C) 2018-2019 Raphael Lehmann <raphael@rleh.de>
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

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

float currentU = 0;
float currentV = 0;

MODM_ISR(TIM1_UP_TIM16)
{
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);

	// Motor phase current measured in phases U,V in low-side
	// Zero current is centered at 0x7ff
	const float adcU = Board::MotorCurrent::AdcU::getValue() - 0x7ff;
	const float adcV = Board::MotorCurrent::AdcV::getValue() - 0x7ff;

	using AdcU = Board::MotorCurrent::AdcU;
	using AdcV = Board::MotorCurrent::AdcV;

	AdcU::acknowledgeInterruptFlags(AdcU::InterruptFlag::EndOfRegularConversion |
		AdcU::InterruptFlag::EndOfSampling | AdcU::InterruptFlag::Overrun);
	AdcV::acknowledgeInterruptFlags(AdcV::InterruptFlag::EndOfRegularConversion |
		AdcV::InterruptFlag::EndOfSampling | AdcV::InterruptFlag::Overrun);

	currentU = adcU; // convertCurrentToA(adcU);
	currentV = adcV; // convertCurrentToA(adcV);

}

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();

	MODM_LOG_ERROR << "Micro-Motor Current Measurement Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::MotorTimer::start();


	using MotorTimer = Board::Motor::MotorTimer;
	MotorTimer::enableInterruptVector(MotorTimer::Interrupt::Update, true, 5);
	MotorTimer::enableInterrupt(MotorTimer::Interrupt::Update);


	while (1)
	{
		Board::Ui::LedRed::toggle();
		Board::Ui::LedGreen::toggle();
		modm::delay_ms(1000);

		MODM_LOG_DEBUG << "current: " << currentU << ", " << currentV << modm::endl;
	}

	return 0;
}
