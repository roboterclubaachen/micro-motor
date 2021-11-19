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
#include <modm/math/filter/pid.hpp>

#include <cmsis/dsp/arm_math.h>

#include <librobots2/motor/bldc_motor_foc.hpp>

#include "svm.hpp"
#include "encoder.hpp"

modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

using namespace std::literals;

modm::PeriodicTimer aliveTimer{100ms};
modm::PeriodicTimer gateDriverStatusTimer{1000ms};
modm::PeriodicTimer debugTimer{1000ms};

modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;

using librobots2::motor::BldcMotorFoc;
using librobots2::motor::setSvmOutput;

BldcMotorFoc<Board::Motor> motor;

namespace
{


/*void setOutputAlphaBeta(float alpha, float beta)
{
	alpha = std::max(std::min(alpha, 1.0f), -1.0f);
	beta = std::max(std::min(beta, 1.0f), -1.0f);

	const auto [a, b, c, valid] = svm(alpha, beta);
	const uint16_t aDutyCycle = std::min<float>(a * Board::Motor::MaxPwm, Board::Motor::MaxPwm);
	const uint16_t bDutyCycle = std::min<float>(b * Board::Motor::MaxPwm, Board::Motor::MaxPwm);
	const uint16_t cDutyCycle = std::min<float>(c * Board::Motor::MaxPwm, Board::Motor::MaxPwm);

	if(debugTimer.execute()) {
		MODM_LOG_DEBUG << "PWM: " << aDutyCycle << ", " << bDutyCycle << ", " << cDutyCycle << "\n";
	}

	Board::Motor::MotorTimer::setCompareValue(1, aDutyCycle);
	Board::Motor::MotorTimer::setCompareValue(2, bDutyCycle);
	Board::Motor::MotorTimer::setCompareValue(3, cDutyCycle);
}*/


constexpr auto clarkeTransform(float u, float v)
	-> std::tuple<float, float>
{
	constexpr float one_by_sqrt3 = 1.f / sqrt(3.f);
	constexpr float two_by_sqrt3 = 2.f / sqrt(3.f);

	return {u, u * one_by_sqrt3 + v * two_by_sqrt3};
}

constexpr float convertCurrentToA(uint16_t adcValue)
{
	constexpr float ShuntResistance = 5e-3;
	constexpr float CurrentGain = 50;
	constexpr float ReferenceVoltage = 2.9;
	constexpr uint16_t AdcCounts = (1 << 12) - 1;

	const float adcVoltage = adcValue * (ReferenceVoltage / AdcCounts);
	const float current = (adcVoltage - (ReferenceVoltage / 2))
		* (1 / (CurrentGain*ShuntResistance));
	return current;
}

/// Align motor to 0°
void alignMotor()
{
	for (int i = 1; i <= 10; ++i) {
		setSvmOutput<Board::Motor>(0.02*i, 0);
		modm::delay_ms(100);
	}
	Board::Encoder::Timer::setValue(0);
}

}

volatile float currentAlpha = 0;
volatile float currentBeta = 0;

using PidParameters = modm::Pid<float>::Parameter;

PidParameters currentControllerParameters = {
	0.004,   // P
	0.01, //0.2,    // I
	0, //0,    // D
	3, //1,  // max I error sum
	0.95  // max output
};

modm::Pid<float> controllerD{currentControllerParameters};
modm::Pid<float> controllerQ{currentControllerParameters};

PidParameters velocityControllerParameters = {
	1.5,   // P
	0.3, //0.2,    // I
	0, //0,    // D
	600, //1,  // max I error sum
	2000  // max output
};

modm::Pid<float> velocityController{velocityControllerParameters};

float commandedCurrentD = 0.0f;
float commandedCurrentQ = 0.0f;
float currentD = 0.0f;
float currentQ = 0.0f;
float voltageAlpha = 0.0f;
float voltageBeta = 0.0f;

constexpr int32_t EncoderTicksPerCycle = 2000;
EncoderAngleEvaluation encoder{EncoderTicksPerCycle};
uint16_t lastEncoderValue = 0;

volatile int16_t velocity = 0;
volatile int16_t targetVelocity = 200;

volatile bool runVelocityControl = false;

MODM_ISR(TIM1_UP_TIM16)
{
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);
	const auto encoderValue = Board::Encoder::getEncoderRaw();
	encoder.update(encoderValue);

	// Run velocity controller at reduced rate
	static uint16_t counter = 0;
	if (counter++ % 200 == 0 && runVelocityControl) {
		const auto currentVelocity = (int16_t)(uint16_t)(encoderValue - lastEncoderValue);
		velocity = currentVelocity;
		lastEncoderValue = encoderValue;
		velocityController.update(currentVelocity - targetVelocity);
		commandedCurrentQ = -velocityController.getValue();
	}

	const auto angleDegrees = encoder.angle() * (360.f / EncoderTicksPerCycle);

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

	const float currentU = adcU; // convertCurrentToA(adcU);
	const float currentV = adcV; // convertCurrentToA(adcV);

	// transform phase currents to alpha/beta coordinates
	const auto [currentAlpha, currentBeta] = clarkeTransform(currentU, currentV);

	motor.setFluxCurrentSetpoint(0);
	motor.setSetpoint(commandedCurrentQ);
	motor.setCurrentMeasurement(currentAlpha, currentBeta);
	motor.setMotorAngle(angleDegrees);
	motor.update();
}

int
main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();

	MODM_LOG_ERROR << "Micro-Motor Gatedriver Test" << modm::endl;

	Board::MotorBridge::GateDriverEnable::set();
	RF_CALL_BLOCKING(gateDriver.initialize());
	RF_CALL_BLOCKING(gateDriver.commit());

	Board::Motor::initialize();
	Board::Motor::MotorTimer::start();

	motor.setControllerParameters(currentControllerParameters);
	motor.enable();


	alignMotor();

	runVelocityControl = true;
	using MotorTimer = Board::Motor::MotorTimer;
	MotorTimer::enableInterruptVector(MotorTimer::Interrupt::Update, true, 5);
	MotorTimer::enableInterrupt(MotorTimer::Interrupt::Update);


	float angle = 0;
	while (1)
	{
		modm::delay_ms(1);
		const float q = velocity;
		const float d = 0;
		modm::can::Message msg(0x100, 8);
		memcpy(&msg.data[0], &q, 4);
		memcpy(&msg.data[4], &d, 4);
		Board::CanBus::Can::sendMessage(msg);

		if(gateDriverStatusTimer.execute()) {
			MODM_LOG_DEBUG << "angle: " << angle << modm::endl;
			MODM_LOG_DEBUG << "current: " << currentAlpha << ", " << currentBeta << modm::endl;
			MODM_LOG_DEBUG << "encoder: " << Board::Encoder::getEncoderRaw() << modm::endl;
		}
	}

	return 0;
}
