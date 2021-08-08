/* main.hpp
 *
 * Copyright (C) 2021 Christopher Durand
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

#include <micro-motor/hardware.hpp>

#include <modm/driver/motor/drv832x_spi.hpp>

#include <cmsis/dsp/arm_math.h>

#include "svm.hpp"

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

modm::PeriodicTimer aliveTimer{100};
modm::PeriodicTimer gateDriverStatusTimer{1000};
modm::PeriodicTimer debugTimer{1000};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;


namespace
{

void setOutput(float magnitude, float angle)
{
    float sine{};
    float cosine{};
    arm_sin_cos_f32(angle, &sine, &cosine);

    sine *= magnitude;
    cosine *= magnitude;

    const auto [a, b, c, valid] = svm(cosine, sine);
	const uint16_t aDutyCycle = std::min<float>(a * Board::Motor::MaxPwm, Board::Motor::MaxPwm);
	const uint16_t bDutyCycle = std::min<float>(b * Board::Motor::MaxPwm, Board::Motor::MaxPwm);
	const uint16_t cDutyCycle = std::min<float>(c * Board::Motor::MaxPwm, Board::Motor::MaxPwm);

	if(debugTimer.execute()) {
		MODM_LOG_DEBUG << "PWM: " << aDutyCycle << ", " << bDutyCycle << ", " << cDutyCycle << "\n";
	}

	Board::Motor::MotorTimer::setCompareValue(1, aDutyCycle);
	Board::Motor::MotorTimer::setCompareValue(2, bDutyCycle);
	Board::Motor::MotorTimer::setCompareValue(3, cDutyCycle);
}

}

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();

	Board::Ui::LedRed::reset();
	Board::Ui::LedGreen::set();
	MODM_LOG_ERROR << "Micro-Motor Gatedriver Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();

	Board::Motor::configure(Board::Motor::Phase::PhaseU, Board::Motor::PhaseConfig::Pwm);
	Board::Motor::configure(Board::Motor::Phase::PhaseV, Board::Motor::PhaseConfig::Pwm);
	Board::Motor::configure(Board::Motor::Phase::PhaseW, Board::Motor::PhaseConfig::Pwm);

	Board::Motor::setCompareValue(0);

    float angle = 0;
	while (1)
	{
		Board::Ui::LedRed::set(/* GateDriverFault */ false);
		modm::delay_ms(1);
		if(aliveTimer.execute()) {
			Board::Ui::LedGreen::toggle();
		}
        angle += 0.5;
		if (angle >= 360) angle -= 360;
		setOutput(0.25, angle);

		if(gateDriverStatusTimer.execute()) {
			MODM_LOG_DEBUG << "angle: " << angle << modm::endl;

			/*RF_CALL_BLOCKING(gateDriver.readAll());
			MODM_LOG_DEBUG << gateDriver.faultStatus1() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.vgsStatus2() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.driverControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveHS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.gateDriveLS() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.ocpControl() << modm::endl;
			MODM_LOG_DEBUG << gateDriver.csaControl() << modm::endl;
			MODM_LOG_ERROR << modm::endl;

			gateDriver.gateDriveHS() &= ~modm::drv832xSpi::HS_IDriveN_t::mask();
			gateDriver.gateDriveHS() |= modm::drv832xSpi::HS_IDriveN_t(modm::drv832xSpi::IDriveN::mA740);*/
		}
	}

	return 0;
}
