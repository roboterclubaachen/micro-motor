/* hardware_rev1.hpp
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

#ifndef HARDWARE_REV1_HPP
#define HARDWARE_REV1_HPP

#include <xpcc/architecture/platform.hpp>

using namespace xpcc::stm32;

namespace Board
{

/// STM32L4 running at 80MHz generated with the PLL from 4MHz MSI clock

struct systemClock {
	static constexpr uint32_t Frequency = 80 * MHz1;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency;
	static constexpr uint32_t Apb2 = Frequency;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart3 = Apb1;
	static constexpr uint32_t Usart4 = Apb1;
	static constexpr uint32_t Usart5 = Apb1;

	static constexpr uint32_t Adc = Frequency;

	static constexpr uint32_t Dac = Frequency;

	static constexpr uint32_t Spi1 = Frequency;
	static constexpr uint32_t Spi2 = Frequency;
	static constexpr uint32_t Spi3 = Frequency;

	static constexpr uint32_t Can1 = Frequency;

	static constexpr uint32_t I2c1 = Frequency;
	//static constexpr uint32_t I2c2 = Frequency;
	static constexpr uint32_t I2c2 = xpcc::clock::MHz48; // TODO real 80MHz, but driver only supports 48MHz
	static constexpr uint32_t I2c3 = Frequency;

	static constexpr uint32_t Timer1  = Frequency;
	static constexpr uint32_t Timer2  = Frequency;
	static constexpr uint32_t Timer6  = Frequency;
	static constexpr uint32_t Timer7  = Frequency;
	static constexpr uint32_t Timer15 = Frequency;
	static constexpr uint32_t Timer16 = Frequency;

	static bool inline
	enable()
	{
		ClockControl::enablePll(
			ClockControl::PllSource::MultiSpeedInternalClock,
			1,	// 4MHz / M=1 -> 4MHz
			40,	// 4MHz * N=40 -> 160MHz <= 344MHz = PLL VCO output max, >= 64 MHz = PLL VCO out min
			2,	// 160MHz / P=2 -> 80MHz = F_cpu
			2	// 80MHz / Q=2 -> 40MHz = F_usb
		);
		ClockControl::setFlashLatency(Frequency);

		// switch system clock to PLL output
		ClockControl::enableSystemClock(ClockControl::SystemClockSource::Pll);
		ClockControl::setAhbPrescaler(ClockControl::AhbPrescaler::Div1);
		// APB1 has max. 80MHz
		ClockControl::setApb1Prescaler(ClockControl::Apb1Prescaler::Div1);
		ClockControl::setApb2Prescaler(ClockControl::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		xpcc::clock::fcpu     = Frequency;
		xpcc::clock::fcpu_kHz = Frequency / 1000;
		xpcc::clock::fcpu_MHz = Frequency / 1000000;
		xpcc::clock::ns_per_loop = ::round(3000.f / (Frequency / 1000000));

		return true;
	}
};

namespace Ui {
	using LedRed		= xpcc::GpioInverted<GpioOutputH0>;
	using LedBlue		= xpcc::GpioInverted<GpioOutputB12>;

	using PadC14		= GpioC14;
	using PadBrk		= GpioA7;
	using PadDac		= GpioA5;

	using DebugUartTx	= GpioOutputA2;
	using DebugUart		= Usart2;
	constexpr uint32_t DebugUartBaudrate = DebugUart::Baudrate::B115200;

	inline void
	initialize()
	{
		LedRed::setOutput(xpcc::Gpio::High);
		LedBlue::setOutput(xpcc::Gpio::High);

		PadC14::setInput(Gpio::InputType::PullUp);

		DebugUartTx::connect(DebugUart::Tx);
		DebugUart::initialize<systemClock, DebugUartBaudrate>(12);
	}
}

namespace Motor {
	using PhaseUN		= GpioOutputB13;
	using PhaseUP		= GpioOutputA8;
	using PhaseVN		= GpioOutputB14;
	using PhaseVP		= GpioOutputA9;
	using PhaseWN		= GpioOutputB15;
	using PhaseWP		= GpioOutputA10;

	using MotorTimer	= Timer1;

	using HallU			= GpioInputB9;
	using HallV			= GpioInputB8;
	using HallW			= GpioInputB6;

	constexpr uint8_t HallInterruptPriority	= 4;
	constexpr uint16_t MaxPwm{0x1FFu}; // 9 bit PWM

	inline void
	initializeMotor()
	{
		MotorTimer::enable();
		MotorTimer::setMode(MotorTimer::Mode::UpCounter,
		                    MotorTimer::SlaveMode::Disabled,
		                    MotorTimer::SlaveModeTrigger::Internal0,
		                    MotorTimer::MasterMode::CompareOc1Ref);
		// MotorTimer clock: APB2 timer clock (180MHz)
		MotorTimer::setPrescaler(1);
		// Prescaler: 1 -> Timer counter frequency: 180MHz
		MotorTimer::setOverflow(MaxPwm); // 9 bit PWM
		// Pwm frequency: 180MHz / 512 = 350kHz
		MotorTimer::configureOutputChannel(1, MotorTimer::OutputCompareMode::Pwm2, 0);
		MotorTimer::configureOutputChannel(2, MotorTimer::OutputCompareMode::Pwm2, 0);
		MotorTimer::configureOutputChannel(3, MotorTimer::OutputCompareMode::Pwm2, 0);
		MotorTimer::enableCaptureComparePreloadedControl();
		MotorTimer::enableOutput();
		MotorTimer::applyAndReset();
		MotorTimer::pause();
		PhaseUN::connect(MotorTimer::Channel1N);
		PhaseVN::connect(MotorTimer::Channel2N);
		PhaseWN::connect(MotorTimer::Channel3N);
		PhaseUP::connect(MotorTimer::Channel1);
		PhaseVP::connect(MotorTimer::Channel2);
		PhaseWP::connect(MotorTimer::Channel3);
	}

	inline void
	initializeHall()
	{
		// Timer is not used for commutation
		// Bldc motor commutation is done using external gpio pin interrupts
		HallU::setInput(Gpio::InputType::PullUp);
		HallV::setInput(Gpio::InputType::PullUp);
		HallW::setInput(Gpio::InputType::PullUp);

		HallU::setInputTrigger(Gpio::InputTrigger::BothEdges);
		HallU::enableExternalInterrupt();
		HallU::enableExternalInterruptVector(HallInterruptPriority);
		HallV::setInputTrigger(Gpio::InputTrigger::BothEdges);
		HallV::enableExternalInterrupt();
		HallV::enableExternalInterruptVector(HallInterruptPriority);
		HallW::setInputTrigger(Gpio::InputTrigger::BothEdges);
		HallW::enableExternalInterrupt();
		HallW::enableExternalInterruptVector(HallInterruptPriority);
	}

	inline void
	setCompareValue(uint16_t compareValue)
	{
		MotorTimer::setCompareValue(1, compareValue);
		MotorTimer::setCompareValue(2, compareValue);
		MotorTimer::setCompareValue(3, compareValue);
	}

	inline void
	initialize()
	{
		initializeHall();
		initializeMotor();
	}
}

namespace MotorBridge {
	using GateDriverEnable	= GpioOutputB1;
	using GateDriverFault	= xpcc::GpioInverted<GpioInputB2>;

	struct GateDriver
	{
		static constexpr auto SpiBaudrate = 5'000'000;

		using Spi	= SpiMaster1;
		using Cs	= GpioOutputA15;
		using Sck	= GpioOutputB3;
		using Miso	= GpioInputB4;
		using Mosi	= GpioOutputB5;

		static inline void
		initialize()
		{
			GateDriver::Cs::setOutput(true);

			Sck::connect(Spi::Sck);
			Mosi::connect(Spi::Mosi);
			Miso::connect(Spi::Miso, Gpio::InputType::PullUp);
			Spi::initialize<systemClock, SpiBaudrate>();
		}
	};

	inline void
	initialize()
	{
		GateDriverEnable::setOutput(Gpio::OutputType::PushPull);
		GateDriverEnable::set(false);

		GateDriverFault::setInput(Gpio::InputType::PullUp);

		GateDriver::initialize();

	}
}

namespace MotorCurrent {
	using SenseCal	= GpioOutputB7;

	using SenseAll	= GpioInputA4;
	using SenseV	= GpioInputA6;
	using SenseW	= GpioInputB0;
	using Adc = Adc1;

	using CurrentAll	= GpioInputA3; // Shorted to GpioA4 on PCB
	using BrakeOut		= GpioOutputA7;
	//using Comp = Comp2;

	inline void
	initialize()
	{
		SenseCal::setOutput(Gpio::OutputType::PushPull);
		SenseCal::set(false);

		Adc::initialize(); // TODO
		SenseAll::connect(Adc::Channel9);
		SenseV::connect(Adc::Channel11);
		SenseW::connect(Adc::Channel15);

		// TODO initialize comparator
	}
}

namespace Encoder {
	using PinA = GpioInputA0;
	using PinB = GpioInputA1;
	using Timer = Timer2;

	inline Timer::Value getEncoderRaw()
	{
		return Timer::getValue();
	}

	inline void
	initialize()
	{
		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter, Timer::SlaveMode::Encoder3);
		// Overflow must be 16bit because else a lot of our motor control code will break!
		Timer::setOverflow(0xffff);
		//PinA::connect(Timer::Channel1, Gpio::InputType::Floating);
		PinB::connect(Timer::Channel2, Gpio::InputType::Floating);
		Timer::start();
	}
}

namespace TemperatureSensor {
	using TemperatureScl	= GpioB10;
	using TemperatureSda	= GpioB11;
	using TemperatureI2c	= I2cMaster2;
	static constexpr uint32_t TemperatureBaudrate = TemperatureI2c::Baudrate::Standard;

	inline void
	initialize()
	{
		TemperatureScl::connect(TemperatureI2c::Scl);
		TemperatureSda::connect(TemperatureI2c::Sda);
		TemperatureI2c::initialize<systemClock, TemperatureBaudrate, xpcc::Tolerance::FivePercent>();
	}
}

namespace CanBus {
	using CanRx	= GpioInputA11;
	using CanTx	= GpioOutputA12;
	using Can = Can1;

	static constexpr uint32_t CanBaudrate = Can::Bitrate::kBps125;

	inline void
	initialize()
	{
		CanRx::connect(Can::Rx, Gpio::InputType::PullUp);
		CanTx::connect(Can::Tx, Gpio::OutputType::PushPull);
		Can::initialize<systemClock, CanBaudrate>(9);
	}
}

inline void
initializeMcu()
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();
}

inline void
initializeAllPeripherals()
{
	Ui::initialize();
	Motor::initialize();
	MotorBridge::initialize();
	MotorCurrent::initialize();
	Encoder::initialize();
	TemperatureSensor::initialize();
	CanBus::initialize();
}

}

#endif // HARDWARE_REV1_HPP
