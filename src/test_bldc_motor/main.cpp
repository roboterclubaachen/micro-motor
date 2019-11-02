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

#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/debug/logger.hpp>

#include "../hardware_rev2.hpp"

#include <modm/driver/motor/drv832x_spi.hpp>

#include <type_traits>

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm::PeriodicTimer aliveTimer{1};
modm::PeriodicTimer gateDriverStatusTimer{5000};

modm::PeriodicTimer adcTimer{10};
uint32_t adcCounter = 0;
constexpr std::size_t adcBufferLength = 50;
uint16_t adcBuffer[adcBufferLength];

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;


// Commutation sequence:
// ..., 1, 3, 2, 6, 4, 5, ...
static constexpr const uint_fast8_t sequenceLut[8] = {
	6, // invalid
	0,			// 0b001
	2,			// 0b010
	1,			// 0b011
	4,			// 0b100
	5,			// 0b101
	3,			// 0b110
	6, // invalid
};

static constexpr const uint_fast8_t commutationLut[7] = {
	0b11'10'00, // hall 0b001 -> 0
	0b11'00'10, // hall 0b011 -> 1
	0b00'11'10, // hall 0b010 -> 2
	0b10'11'00, // hall 0b110 -> 3
	0b10'00'11, // hall 0b100 -> 4
	0b00'10'11, // hall 0b101 -> 5
	0b00'00'00, // hall invalid -> 6
};

uint32_t commutationOffset = 1; // 0..5

volatile bool reverse = false;
volatile bool disable = false;

void
doCommutation()
{
	uint_fast8_t hall = Board::Motor::HallPort::read();
	uint32_t index;
	if(disable) {
		index = 6;
	}
	else {
		if(reverse) {
			// Bitwise invert hall pins
			hall ^= 0b111;
			index = sequenceLut[hall] + (6 - commutationOffset);
		}
		else {
			index = sequenceLut[hall] + commutationOffset;
		}
		while(index >= 6) { index -= 6; }
	}
	uint_fast8_t out = commutationLut[index];

	switch(out & 0b11) {
		case 0b00:
		/*case 0b01:*/
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseU, Board::Motor::PhaseOutputConfig::HiZ);
			break;
		case 0b10:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseU, Board::Motor::PhaseOutputConfig::Low);
			break;
		case 0b11:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseU, Board::Motor::PhaseOutputConfig::NormalPwm);
			break;
	}
	switch((out >> 2) & 0b11) {
		case 0b00:
		/*case 0b01:*/
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseV, Board::Motor::PhaseOutputConfig::HiZ);
			break;
		case 0b10:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseV, Board::Motor::PhaseOutputConfig::Low);
			break;
		case 0b11:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseV, Board::Motor::PhaseOutputConfig::NormalPwm);
			break;
	}
	switch((out >> 4) & 0b11) {
		case 0b00:
		/*case 0b01:*/
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseW, Board::Motor::PhaseOutputConfig::HiZ);
			break;
		case 0b10:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseW, Board::Motor::PhaseOutputConfig::Low);
			break;
		case 0b11:
			Board::Motor::configurePhase(Board::Motor::Phase::PhaseW, Board::Motor::PhaseOutputConfig::NormalPwm);
			break;
	}

	// Generate update event
	Board::Motor::MotorTimer::generateEvent(Board::Motor::MotorTimer::Event::CaptureCompareControlUpdate);

	//Board::Ui::LedBlue::toggle();
}

MODM_ISR(EXTI15_10)
{
	static_assert(std::is_same<Board::Motor::HallU, GpioC13>::value, "Commutation Interrupt assumes HallU is GpioC13");
	static_assert(std::is_same<Board::Motor::HallV, GpioC14>::value, "Commutation Interrupt assumes HallV is GpioC14");
	static_assert(std::is_same<Board::Motor::HallW, GpioC15>::value, "Commutation Interrupt assumes HallW is GpioC15");
	Board::Motor::HallU::acknowledgeExternalInterruptFlag();
	Board::Motor::HallV::acknowledgeExternalInterruptFlag();
	Board::Motor::HallW::acknowledgeExternalInterruptFlag();
	doCommutation();
}

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::Led::reset();
	MODM_LOG_ERROR << "Micro-Motor BLDC Motor Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	gateDriver.csaControl().set(modm::drv832xSpi::CsaControl::CsaFet);
	gateDriver.csaControl().set(modm::drv832xSpi::CsaControl::LowSideReference);
	gateDriver.csaControl() &= ~modm::drv832xSpi::CsaGain_t::mask();
	gateDriver.csaControl() |= modm::drv832xSpi::CsaGain_t(modm::drv832xSpi::CsaGain::Gain40);
	RF_CALL_BLOCKING(gateDriver.commit());

	MODM_LOG_ERROR << "Gate driver initialized" << modm::endl;

	Board::Motor::initializeHall();

	Board::Motor::MotorTimer::start();
	Board::Motor::MotorTimer::enableOutput();
	Board::Motor::setCompareValue(Board::Motor::MaxPwm * 16 / 32);
	Board::Motor::MotorTimer::applyAndReset();
	// initial commutation
	doCommutation();

	MODM_LOG_ERROR << "Motor timer initialized" << modm::endl;

	Board::MotorCurrent::Adc::startConversion();

	MODM_LOG_ERROR << "Adc timer initialized" << modm::endl;

	while (1)
	{
		Board::Ui::Led::set(Board::MotorBridge::GateDriverFault::read());
		//modm::delayMilliseconds(1);

		/*if(aliveTimer.execute()) {
			//Board::Ui::LedBlue::toggle();
			static bool toggle = false;
			Board::Motor::setCompareValue(toggle ? 0 : Board::Motor::MaxPwm * 16 / 32);
			toggle = !toggle;
		}*/

		/*if(adcTimer.execute()) {
			if(Adc1::isConversionFinished()) {
				adcBuffer[adcCounter] = Board::MotorCurrent::Adc::getValue();
				adcCounter++;
				// Start next conversion
				Board::MotorCurrent::Adc::startConversion();
			}
			else {
				MODM_LOG_DEBUG << "ADC conversion not finished." << modm::endl;
			}
			if(adcCounter == adcBufferLength) {
				MODM_LOG_DEBUG << "ADC=[ ";
				uint32_t sum = 0;
				for(std::size_t i = 0; i < adcBufferLength; i++) {
					sum += adcBuffer[i];
					MODM_LOG_DEBUG << adcBuffer[i] << " ";
				}
				float current = ((2048.f - (sum / adcBufferLength)) * 0.000805664062  / 0.005  / 40.f) - 0.185;
				MODM_LOG_DEBUG << "]; sum=" << sum << " current=";
				MODM_LOG_DEBUG.printf("%1.3f", current);
				MODM_LOG_DEBUG << modm::endl;
				adcCounter = 0;
			}
		} */

		if(gateDriverStatusTimer.execute()) {
			RF_CALL_BLOCKING(gateDriver.readAll());
			MODM_LOG_DEBUG << gateDriver.faultStatus1() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.vgsStatus2() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.driverControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveHS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveLS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.ocpControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.csaControl() << modm::endl;
			MODM_LOG_ERROR << modm::endl;
		}

		static bool toggle = false;
		Board::Motor::setCompareValue(toggle ? 0 : Board::Motor::MaxPwm * 24 / 32);
		toggle = !toggle;
		modm::delayMicroseconds(50);
	}
	return 0;
}
