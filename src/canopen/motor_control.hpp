#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#include <modm-canopen/canopen_device.hpp>
#include "motor_state.hpp"

#include "pwm_protocol.hpp"
#include "velocity_protocol.hpp"
#include "position_protocol.hpp"

struct CanopenObjects
{
	static constexpr modm_canopen::Address ControlWord{0x6040, 0};
	static constexpr modm_canopen::Address StatusWord{0x6041, 0};
	static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
	static constexpr modm_canopen::Address ModeOfOperationDisplay{0x6061, 0};
};

struct FactorObjects
{
	static constexpr modm_canopen::Address PositionFactorNumerator{0x6093, 1};
	static constexpr modm_canopen::Address PositionFactorDivisor{0x6093, 2};
	static constexpr modm_canopen::Address VelocityFactorNumerator{0x6094, 1};
	static constexpr modm_canopen::Address VelocityFactorDivisor{0x6094, 2};
	static constexpr modm_canopen::Address AccelerationFactorNumerator{0x6097, 1};
	static constexpr modm_canopen::Address AccelerationFactorDivisor{0x6097, 2};
	static constexpr modm_canopen::Address Polarity{0x607E, 0};
};

template<typename... Modes>
class MotorControl
{
private:
	static inline MotorState state_{};

	template<typename First, typename Second, typename... Rest>
	static bool
	updateMode();

	template<typename First>
	static bool
	updateMode();

public:
	static inline const MotorState&
	state()
	{
		return state_;
	}

	static bool
	update();

	static inline void
	setActualPosition(int32_t position)
	{
		state_.actualPosition_ = position;
	}

	static inline int16_t
	outputPWM()
	{
		return state_.outputPWM_;
	}

	template<typename ObjectDictionary>
	constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);
};

using MotorControl0 =
	MotorControl<PWMProtocol, VelocityProtocol, PositionProtocol<VelocityProtocol>>;

#include "motor_control_impl.hpp"
#endif