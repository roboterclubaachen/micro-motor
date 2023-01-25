#ifndef POSITION_PROTOCOL_HPP
#define POSITION_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/pid.hpp>

#include "motor_state.hpp"
using Pid = modm::Pid<float>;

struct PositionObjects
{
	static constexpr modm_canopen::Address PositionDemandValue{0x6062, 0};        // User units
	static constexpr modm_canopen::Address PositionInternalValue{0x6063, 0};      // internal units
	static constexpr modm_canopen::Address PositionActualValue{0x6064, 0};        // User units
	static constexpr modm_canopen::Address TargetPosition{0x607A, 0};             // User units
	static constexpr modm_canopen::Address PositionWindow{0x6067, 0};             // User units
	static constexpr modm_canopen::Address FollowingErrorActualValue{0x60F4, 0};  // User units

	static constexpr modm_canopen::Address PositionPID_kP{0x2006, 1};           // Custom
	static constexpr modm_canopen::Address PositionPID_kI{0x2006, 2};           // Custom
	static constexpr modm_canopen::Address PositionPID_kD{0x2006, 3};           // Custom
	static constexpr modm_canopen::Address PositionPID_MaxErrorSum{0x2006, 4};  // Custom
};

template<typename VelocityProtocol>
class PositionProtocol
{
public:
	static inline Pid::Parameter positionPidParameters_{1.0f, 0.0f, 0.0f, 100.0f,
														std::numeric_limits<int16_t>::max()};
	static inline Pid positionPid_;
	static inline bool receivedPositionRelative_{true};
	static inline int32_t receivedPosition_{};
	static inline int32_t nextPosition_{};
	static inline int32_t commandedPosition_{};
	static inline uint32_t positionWindow_{5};
	static inline int32_t positionError_{};
	static inline uint32_t positionWindowTime_{10};
	static inline uint32_t inPositionWindow_{0};

public:
	static bool
	applicable(const MotorState& state)
	{
		return state.mode_ == OperatingMode::Position &&
			   state.status_.state() == modm_canopen::cia402::State::OperationEnabled;
	}

	static inline bool
	update(MotorState& state);

	template<typename ObjectDictionary, const MotorState& state>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map);
};

#include "position_protocol_impl.hpp"
#endif