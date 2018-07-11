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

#include "../hardware_rev1.hpp"

#include <modm/driver/motor/drv832x_spi.hpp>

#include <type_traits>

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

//modm::PeriodicTimer aliveTimer{100};
modm::PeriodicTimer gateDriverStatusTimer{50000};

modm::PeriodicTimer adcTimer{10};
uint32_t adcCounter = 0;
constexpr std::size_t adcBufferLength = 50;
uint16_t adcBuffer[adcBufferLength];

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;


static constexpr const uint_fast8_t outputLutI[8] = {
	0b00'00'00, /* stop/disable */
	0b11'00'10,
	0b00'10'11,
	0b11'10'00,
	0b10'11'00,
	0b00'11'10,
	0b10'00'11,
	0, /* invalid */
};
static constexpr const uint_fast8_t outputLutJ[8] = {
	0b00'00'00, /* stop/disable */
	0b10'11'00,
	0b11'00'10,
	0b00'11'10,
	0b00'10'11,
	0b10'00'11,
	0b11'10'00,
	0, /* invalid */
};
static constexpr const uint_fast8_t outputLutK[8] = {
	0b00'00'00, /* stop/disable */
	0b00'10'11,
	0b10'11'00,
	0b10'00'11,
	0b11'00'10,
	0b11'10'00,
	0b00'11'10,
	0, /* invalid */
};
static constexpr const uint_fast8_t outputLutL[8] = {
	0b00'00'00, /* stop/disable */
	0b11'10'00,
	0b00'11'10,
	0b11'00'10,
	0b10'00'11,
	0b00'10'11,
	0b10'11'00,
	0, /* invalid */
};
static constexpr const uint_fast8_t outputLutM[8] = {
	0b00'00'00, /* stop/disable */
	0b10'00'11,
	0b11'10'00,
	0b00'10'11,
	0b00'11'10,
	0b10'11'00,
	0b11'00'10,
	0, /* invalid */
};
static constexpr const uint_fast8_t outputLutN[8] = {
	0b00'00'00, /* stop/disable */
	0b00'11'10,
	0b10'00'11,
	0b10'11'00,
	0b11'10'00,
	0b11'00'10,
	0b00'10'11,
	0, /* invalid */
};
static constexpr const uint_fast8_t* outputLutLut[6] = {
	outputLutI,
	outputLutJ,
	outputLutK,
	outputLutL,
	outputLutM,
	outputLutN,
};

enum class
HallPermutation : std::size_t {
	PermutationI = 0,
	PermutationJ = 1,
	PermutationK = 2,
	PermutationL = 3,
	PermutationM = 4,
	PermutationN = 5,
};

HallPermutation hallPermutation = HallPermutation::PermutationL;

void modm_always_inline
doCommutation()
{
	uint_fast8_t hall = Board::Motor::HallPort::read();
	if(false/*reverse*/) {
		// Bitwise invert hall pins
		hall ^= 0b111;
	}
	if(false/*stop/disable*/) {
		// Illegal hall sensor state
		hall = 0b000;
	}
	uint_fast8_t out = outputLutLut[static_cast<std::size_t>(hallPermutation)][hall];

	MODM_LOG_ERROR << "h=" << hall <<  " o=" << modm::bin << static_cast<char>(out) << modm::endl;

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

	Board::Ui::LedBlue::toggle();
}

MODM_ISR(EXTI9_5)
{
	static_assert(std::is_same<Board::Motor::HallU, GpioB9>::value, "Commutation Interrupt assumes HallU is GpioB9");
	static_assert(std::is_same<Board::Motor::HallV, GpioB8>::value, "Commutation Interrupt assumes HallV is GpioB8");
	static_assert(std::is_same<Board::Motor::HallW, GpioB6>::value, "Commutation Interrupt assumes HallW is GpioB6");
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

	Board::Ui::LedRed::reset();
	Board::Ui::LedBlue::set();
	MODM_LOG_ERROR << "Micro-Motor BLDC Motor Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	gateDriver.csaControl() &= ~modm::drv832xSpi::CsaGain_t::mask();
	gateDriver.csaControl() |= modm::drv832xSpi::CsaGain_t(modm::drv832xSpi::CsaGain::Gain40);
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initializeHall();

	Board::Motor::MotorTimer::start();
	Board::Motor::MotorTimer::enableOutput();
	Board::Motor::setCompareValue(Board::Motor::MaxPwm * 1 / 8);
	Board::Motor::MotorTimer::applyAndReset();
	// initial commutation
	doCommutation();

	Board::MotorCurrent::Adc::startConversion();

	while (1)
	{
		Board::Ui::LedRed::set(Board::MotorBridge::GateDriverFault::read());
		modm::delayMilliseconds(1);

		/*if(aliveTimer.execute()) {
			Board::Ui::LedBlue::toggle();
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
		}*/

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
	}
	return 0;
}
