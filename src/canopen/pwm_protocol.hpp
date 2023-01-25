#ifndef PWM_PROTOCOL_HPP
#define PWM_PROTOCOL_HPP
#include <cstdint>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/states.hpp>

#include "motor_state.hpp"

struct PWMObjects
{
	static constexpr modm_canopen::Address PWMCommand{0x2002, 0};  // Custom
	static constexpr modm_canopen::Address OutputPWM{0x2003, 0};   // Custom
};

class PWMProtocol
{
public:
	static inline int16_t commandedPWM_{0};

public:
	static bool
	applicable(const MotorState& state)
	{
		return state.mode_ == OperatingMode::Voltage &&
			   state.status_.state() == modm_canopen::cia402::State::OperationEnabled;
	}

	static inline bool
	update(MotorState& state);

	template<typename ObjectDictionary, const MotorState& state>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);
};

#include "pwm_protocol_impl.hpp"
#endif