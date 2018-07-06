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

#include <modm/platform/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/platform/clock/clock.hpp>

using namespace modm::platform;

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
	static constexpr uint32_t I2c2 = MHz48; // TODO real 80MHz, but driver only supports 48MHz
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
		ClockControl::enableInternalClock();

		ClockControl::enablePll(
			ClockControl::PllSource::Hsi,
			1,	// 16Hz / M=1 -> 16MHz
			10,	// 16MHz * N=10 -> 160MHz <= 344MHz = PLL VCO output max, >= 64 MHz = PLL VCO out min
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
		modm::clock::fcpu     = Frequency;
		modm::clock::fcpu_kHz = Frequency / 1000;
		modm::clock::fcpu_MHz = Frequency / 1000000;
		modm::clock::ns_per_loop = ::round(3000.f / (Frequency / 1000000));

		return true;
	}
};

namespace Ui {
	using LedRed		= GpioInverted<GpioH0>;
	using LedBlue		= GpioInverted<GpioB12>;

	using PadC14		= GpioC14;
	using PadBrk		= GpioA7;
	using PadDac		= GpioA5;

	using DebugUartTx	= GpioA2;
	using DebugUart		= Usart2;
	static constexpr uint32_t DebugUartBaudrate = DebugUart::Baudrate::B115200 * 4; // 460800 baud

	inline void
	initialize()
	{
		LedRed::setOutput(modm::Gpio::High);
		LedBlue::setOutput(modm::Gpio::High);

		PadC14::setInput(Gpio::InputType::PullUp);

		DebugUart::connect<DebugUartTx::Tx>();
		DebugUart::initialize<systemClock, DebugUartBaudrate>(12);
	}
}

namespace Motor {
	using PhaseUN		= GpioB13;
	using PhaseUP		= GpioA8;
	using PhaseVN		= GpioB14;
	using PhaseVP		= GpioA9;
	using PhaseWN		= GpioB15;
	using PhaseWP		= GpioA10;

	using MotorTimer	= Timer1;

	using HallU			= GpioB9;
	using HallV			= GpioB8;
	using HallW			= GpioB6;

	constexpr uint8_t HallInterruptPriority	= 4;
	constexpr uint16_t MaxPwm{0x2FFu}; // 10 bit PWM

	inline void
	initializeMotor()
	{
		MotorTimer::enable();
		MotorTimer::setMode(MotorTimer::Mode::UpCounter);

		// MotorTimer clock: APB2 timer clock (80MHz)
		MotorTimer::setPrescaler(1);
		// Prescaler: 1 -> Timer counter frequency: 80MHz
		MotorTimer::setOverflow(MaxPwm);
		// Pwm frequency: 80MHz / 1024 = 78kHz

		MotorTimer::configureOutputChannel(1,
										   MotorTimer::OutputCompareMode::Pwm,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::OutputComparePreload::Disable
										   );
		MotorTimer::configureOutputChannel(2,
										   MotorTimer::OutputCompareMode::Pwm,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::OutputComparePreload::Disable
										   );
		MotorTimer::configureOutputChannel(3,
										   MotorTimer::OutputCompareMode::Pwm,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::PinState::Enable,
										   MotorTimer::OutputComparePolarity::ActiveHigh,
										   MotorTimer::OutputComparePreload::Disable
										   );
		MotorTimer::setCompareValue(1, 0);
		MotorTimer::setCompareValue(2, 0);
		MotorTimer::setCompareValue(3, 0);

		MotorTimer::applyAndReset();
		MotorTimer::enableOutput();

		MotorTimer::pause();

		MotorTimer::connect<PhaseUN::Ch1n,
		                    PhaseVN::Ch2n,
		                    PhaseWN::Ch3n,
		                    PhaseUP::Ch1,
		                    PhaseVP::Ch2,
		                    PhaseWP::Ch3>();
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
		//initializeHall();
		initializeMotor();
	}
}

namespace MotorBridge {
	using GateDriverEnable	= GpioB1;
	using GateDriverFault	= GpioInverted<GpioB2>;

	struct GateDriver
	{
		static constexpr auto SpiBaudrate = 312500;

		using Spi	= SpiMaster1;
		using Cs	= GpioA15;
		using Sck	= GpioB3;
		using Miso	= GpioB4;
		using Mosi	= GpioB5;

		static inline void
		initialize()
		{
			GateDriver::Cs::setOutput(true);

			Spi::connect<Sck::Sck, Mosi::Mosi, Miso::Miso>();

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
	using SenseCal	= GpioB7;

	using SenseAll	= GpioA4;
	using SenseV	= GpioA6;
	using SenseW	= GpioB0;
	using Adc = Adc1;

	using CurrentAll	= GpioA3; // Shorted to GpioA4 on PCB
	using BrakeOut		= GpioA7;
	//using Comp = Comp2;

	inline void
	initialize()
	{
		SenseCal::setOutput(Gpio::OutputType::PushPull);
		SenseCal::set(false);

		Adc::initialize(); // TODO
		Adc::connect<SenseAll::In9, SenseV::In11, SenseW::In15>();

		// TODO initialize comparator
	}
}

namespace Encoder {
	using PinA = GpioA0;
	using PinB = GpioA1;
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

		Timer::connect<PinA::Ch1, PinB::Ch2>();

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
		TemperatureI2c::connect<TemperatureScl::Scl, TemperatureSda::Sda>();
		TemperatureI2c::initialize<systemClock, TemperatureBaudrate, modm::Tolerance::FivePercent>();
	}
}

namespace CanBus {
	using CanRx	= GpioA11;
	using CanTx	= GpioA12;
	using Can = Can1;

	static constexpr uint32_t CanBaudrate = Can::Bitrate::kBps125;

	inline void
	initialize()
	{
		Can::connect<CanRx::Rx, CanTx::Tx>(Gpio::InputType::PullUp);
		Can::initialize<systemClock, CanBaudrate>(9);
	}
}

inline void
initializeMcu()
{
	systemClock::enable();
	modm::cortex::SysTickTimer::initialize<systemClock>();
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
