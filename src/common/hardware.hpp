/* Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
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

#ifndef HARDWARE_V2_HPP
#define HARDWARE_V2_HPP

#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/platform/uart/uart_1.hpp>

using namespace modm::platform;

namespace Board
{
	using namespace modm::literals;

/// STM32G474RE running at 170MHz generated from the internal 16MHz crystal
// Dummy clock for devices
struct SystemClock {
	static constexpr uint32_t Frequency = 170_MHz;
	static constexpr uint32_t Ahb1      = Frequency;
	static constexpr uint32_t Ahb2      = Frequency;
	static constexpr uint32_t Apb1      = Frequency;
	static constexpr uint32_t Apb2      = Frequency;

	static constexpr uint32_t Cordic    = Ahb1;
	static constexpr uint32_t Crc       = Ahb1;
	static constexpr uint32_t Dma       = Ahb1;
	static constexpr uint32_t Dma1      = Dma;
	static constexpr uint32_t Dma2      = Dma;
	static constexpr uint32_t DmaMux    = Dma;
	static constexpr uint32_t Fmac      = Ahb1;

	static constexpr uint32_t Adc       = Ahb2;
	static constexpr uint32_t Adc1      = Adc;
	static constexpr uint32_t Adc2      = Adc;
	static constexpr uint32_t Adc3      = Adc;
	static constexpr uint32_t Adc4      = Adc;
	static constexpr uint32_t Adc5      = Adc;
	static constexpr uint32_t Dac       = Ahb2;
	static constexpr uint32_t Dac1      = Dac;
	static constexpr uint32_t Dac2      = Dac;
	static constexpr uint32_t Dac3      = Dac;
	static constexpr uint32_t Dac4      = Dac;
	static constexpr uint32_t Rng       = Ahb2;

	static constexpr uint32_t Can       = Apb1;
	static constexpr uint32_t Fdcan1    = Can;
	static constexpr uint32_t Fdcan2    = Can;
	static constexpr uint32_t Fdcan3    = Can;
	static constexpr uint32_t I2c       = Apb1;
	static constexpr uint32_t I2c1      = I2c;
	static constexpr uint32_t I2c2      = I2c;
	static constexpr uint32_t I2c3      = I2c;
	static constexpr uint32_t I2c4      = I2c;
	static constexpr uint32_t Lptim     = Apb1;
	static constexpr uint32_t Lpuart    = Apb1;
	static constexpr uint32_t Rtc       = Apb1;
	static constexpr uint32_t Spi2      = Apb1;
	static constexpr uint32_t Spi3      = Apb1;
	static constexpr uint32_t Uart4     = Apb1;
	static constexpr uint32_t Uart5     = Apb1;
	static constexpr uint32_t Usart2    = Apb1;
	static constexpr uint32_t Usart3    = Apb1;
	static constexpr uint32_t Usb       = Apb1;
	static constexpr uint32_t Apb1Timer = Apb1 * 1;
	static constexpr uint32_t Timer2    = Apb1Timer;
	static constexpr uint32_t Timer3    = Apb1Timer;
	static constexpr uint32_t Timer4    = Apb1Timer;
	static constexpr uint32_t Timer5    = Apb1Timer;
	static constexpr uint32_t Timer6    = Apb1Timer;
	static constexpr uint32_t Timer7    = Apb1Timer;

	static constexpr uint32_t Sai1      = Apb2;
	static constexpr uint32_t Spi1      = Apb2;
	static constexpr uint32_t Usart1    = Apb2;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1    = Apb2Timer;
	static constexpr uint32_t Timer8    = Apb2Timer;
	static constexpr uint32_t Timer15   = Apb2Timer;
	static constexpr uint32_t Timer16   = Apb2Timer;
	static constexpr uint32_t Timer17   = Apb2Timer;
	static constexpr uint32_t Timer20   = Apb2Timer;

	static bool inline
	enable()
	{
		Rcc::enableInternalClock();	// 16MHz
		Rcc::enablePll(
			Rcc::PllSource::InternalClock,
			4,	//  16MHz / M= 4 -> 4MHz
			85,	//   4MHz * N=85 -> 340MHz
			2	// 340MHz / R= 2 -> 170MHz = F_cpu
		);
		// set flash latency for 170MHz
		Rcc::setFlashLatency<Frequency>();
		// switch system clock to PLL output
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		// APB1 has max. 170MHz
		Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
		Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};

namespace Ui {
	using LedRed		= GpioA4; // DAC1
	using LedGreen		= GpioA5; // DAC2

	using DebugUartTx	= GpioB6;
	using DebugUartRx	= GpioB7;
	using DebugUart		= Usart1;
	static constexpr uint32_t DebugUartBaudrate = 460800_Bd;

	inline void
	initialize()
	{
		LedRed::setOutput(false);
		LedGreen::setOutput(false);

		DebugUart::connect<DebugUartTx::Tx, DebugUartRx::Rx>();
		DebugUart::initialize<SystemClock, DebugUartBaudrate>(12);
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

	using HallU			= GpioC13;
	using HallV			= GpioC14;
	using HallW			= GpioC15;
	using HallPort		= SoftwareGpioPort<HallU, HallV, HallW>;

//	constexpr uint8_t HallInterruptPriority	= 4; // Please don't use pin change interrupts on hall pins! Danger!

	inline void
	setCompareValue(uint16_t compareValue)
	{
		MotorTimer::setCompareValue(1, compareValue);
		MotorTimer::setCompareValue(2, compareValue);
		MotorTimer::setCompareValue(3, compareValue);
	}

	constexpr uint16_t MaxPwm{511u}; // 9 bit PWM

	enum class
	PhaseOutputConfig : uint32_t
	{
		HiZ,
		NormalPwm,
		High,
		Low,
	};

	enum class
	Phase : uint32_t
	{
		PhaseU = 1,
		PhaseV = 2,
		PhaseW = 3,
	};

	void
	configurePhase(Phase phase, PhaseOutputConfig phaseOutputConfig)
	{
		switch(phaseOutputConfig) {
			case PhaseOutputConfig::HiZ:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase),
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseOutputConfig::NormalPwm:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase),
						MotorTimer::OutputCompareMode::Pwm,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseOutputConfig::High:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase),
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseOutputConfig::Low:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase),
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
		}
	}

	void
	initializeMotor()
	{
		MotorTimer::enable();
		MotorTimer::setMode(MotorTimer::Mode::UpCounter);

		// MotorTimer clock: APB2 timer clock (80MHz)
		MotorTimer::setPrescaler(1);
		// Prescaler: 1 -> Timer counter frequency: 80MHz
		MotorTimer::setOverflow(MaxPwm);
		// Pwm frequency: 80MHz / 1024 = 78kHz

		configurePhase(Phase::PhaseU, PhaseOutputConfig::HiZ);
		configurePhase(Phase::PhaseV, PhaseOutputConfig::HiZ);
		configurePhase(Phase::PhaseW, PhaseOutputConfig::HiZ);

		setCompareValue(0);

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

/*
	inline void
	initializeHall()
	{
		static_assert(HallPort::number_of_ports == 1, "Hall pins must be at the same port to guarantee atomic read operations.");

		// Timer is not used for commutation
		// Bldc motor commutation is done using external gpio pin interrupts
		//HallPort::setInput(Gpio::InputType::PullUp);

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
	initialize()
	{
		//initializeHall();
		initializeMotor();
	}
*/
}

namespace MotorBridge {
	using GateDriverEnable	= GpioF0;

	struct GateDriver
	{
		static constexpr auto SpiBaudrate = 100'000;

		using Cs	= GpioF1;
		using Sck	= GpioB12;
		using Miso	= GpioB11;
		using Mosi	= GpioB10;

		using Spi	= UartSpiMaster3;

		static inline void
		initialize()
		{
			Cs::setOutput(true);
			Spi::connect<Sck::Ck, Mosi::Tx, Miso::Rx>();
			Spi::initialize<SystemClock, SpiBaudrate>();
		}
	};

	inline void
	initialize()
	{
		GateDriverEnable::setOutput(Gpio::OutputType::PushPull);
		GateDriverEnable::set(false);
		GateDriver::initialize();

	}
}

namespace BackEmf {
	using BemfU		= GpioB0;
	using BemfV		= GpioA7;
	using BemfW		= GpioA5;

	// ... (TODO)
	inline void
	initialize()
	{
		// TODO
		BemfU::setInput(Gpio::InputType::Floating);
		BemfV::setInput(Gpio::InputType::Floating);
		BemfW::setInput(Gpio::InputType::Floating);
	}
}

namespace MotorCurrent {
	using SenseU	= GpioA0;
	using SenseV	= GpioA1;
	using SenseW	= GpioA3;
	using Adc = Adc1;

	using CompU = Comp3;
	using CompV = Comp1;
	using CompW = Comp2;

	inline void
	initialize()
	{
		SenseU::setAnalogInput();
		SenseV::setAnalogInput();
		SenseW::setAnalogInput();

		// TODO: Connect DAC3_OUT1 to COMP1_INM
		// TODO: Connect DAC3_OUT2 to COMP2_INM
		// TODO: Connect DAC3_OUT1 to COMP3_INM

		// TODO initialize comparator
		// TODO: Connect COMP{1,2,3}_OUT to TIM1_BKIN

		// initialize STM32 internal DAC3
		Rcc::enable<Peripheral::Dac3>();
		//RCC->AHB2ENR1 |= RCC_AHB2ENR_DAC3EN;

		DAC3->MCR = DAC_MCR_MODE1_0 | DAC_MCR_MODE2_0;
		DAC3->CR = DAC_CR_EN1 | DAC_CR_EN2;
		DAC3->DHR12R1 = 0xFFF / 2;
		DAC3->DHR12R2 = 0xFFF / 2;

		Adc::initialize(Adc::ClockMode::SynchronousPrescaler1,
						Adc::ClockSource::SystemClock,
						Adc::Prescaler::Disabled,
						Adc::CalibrationMode::SingleEndedInputsMode, true);
		Adc::connect<SenseU::In1, SenseV::In2, SenseW::In4>();
		// TODO: Sample ADC
	}
}

namespace Encoder {
	using PinA			= GpioB5;
	using PinB			= GpioB4;
	using PinIndex		= GpioB3;
	using Timer			= Timer3;

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

		Timer::connect<PinA::Ch2, PinB::Ch1>();

		Timer::start();
	}
}

namespace Sensor {
	using Switch		= GpioA15;

	//using Adc 			= Adc??
	using Ntc			= GpioB1; // ADC1_IN12 or ADC3_IN1
	using Vsupply		= GpioB2; // ADC2_IN12

	inline void
	initialize()
	{
		// external pull-up
		Switch::setInput(Gpio::InputType::Floating);
		Ntc::setAnalogInput();
		Vsupply::setAnalogInput();
		
		// TODO: Adc...
	}
}

/*
namespace CanBus {
	using CanRx			= GpioB8;
	using CanTx			= GpioB9;
	using Can			= Fdcan1;

	static constexpr uint32_t CanBaudrate = 1_Mbps;

	inline void
	initialize()
	{
		Can::connect<CanRx::Rx, CanTx::Tx>(Gpio::InputType::PullUp);
		Can::initialize<SystemClock, CanBaudrate>(9);
	}
}
*/

inline void
initializeMcu()
{
	SystemClock::enable();
	modm::cortex::SysTickTimer::initialize<SystemClock>();
}

inline void
initializeAllPeripherals()
{
	Ui::initialize();
	//Motor::initialize();
	MotorBridge::initialize();
	MotorCurrent::initialize();
	Encoder::initialize();
	Sensor::initialize();
	//CanBus::initialize();
}

}

#endif
