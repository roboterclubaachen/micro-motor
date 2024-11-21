#pragma once
#include <cstdint>
#include <limits>
#include <modm-canopen/generated/micro-motor_od.hpp>

#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include <librobots2/motor-canopen/current_protocol.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <librobots2/motor-canopen/pwm_protocol.hpp>
#include <librobots2/motor-canopen/error_protocol.hpp>
#include <librobots2/motor-canopen/velocity_protocol.hpp>
#include <librobots2/motor-canopen/position_protocol.hpp>
#include <librobots2/motor-canopen/quickstop_protocol.hpp>
#include <librobots2/motor-canopen/encoder_protocol.hpp>
#include <librobots2/motor-canopen/motor_state.hpp>

template<size_t id, typename Encoder>
using MotorControl_t =
	MotorControl<id, MotorState<id>, PWMProtocol<id>,
				 VelocityProtocol<id>, PositionProtocol<id>, QuickstopProtocol<id>,
				 CurrentProtocol<id>, EncoderProtocol<id, Encoder>>;
