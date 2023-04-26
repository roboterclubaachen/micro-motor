#ifndef VELOCITY_PROTOCOL_HPP
#define VELOCITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/pid.hpp>

#include "motor_state.hpp"
using Pid = modm::Pid<float>;

struct VelocityObjects {
  static constexpr modm_canopen::Address VelocityDemandValue{0x606B,
                                                             0}; // User units
  static constexpr modm_canopen::Address TargetVelocity{0x60FF,
                                                        0}; // User units
  static constexpr modm_canopen::Address ProfileAcceleration{0x6083,
                                                             0}; // User units

  static constexpr modm_canopen::Address VelocityError{0x2004, 1}; // Custom

  static constexpr modm_canopen::Address VelocityPID_kP{0x2005, 1}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kI{0x2005, 2}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kD{0x2005, 3}; // Custom
  static constexpr modm_canopen::Address VelocityPID_MaxErrorSum{0x2005,
                                                                 4}; // Custom
};

template <size_t id> class VelocityProtocol {
public:
  static inline Pid::Parameter velocityPidParameters_{
      1.0f, 0.0f, 0.0f, 100.0f, std::numeric_limits<int16_t>::max()};
  static inline Pid velocityPid_;
  static inline int32_t receivedVelocity_{};
  static inline int32_t commandedVelocity_{};
  static inline int32_t velocityError_{};

  static inline int32_t profileAcceleration_{5000};

public:
  static bool applicable(const MotorState &state) {
    return state.enableMotor_ && state.mode_ == OperatingMode::Velocity &&
           state.status_.state() ==
               modm_canopen::cia402::State::OperationEnabled;
  }

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  static inline int16_t doPositionUpdate(int32_t commandedVelocity,
                                         const MotorState &state);

  static inline int16_t doQuickStopUpdate(int32_t commandedDeceleration,
                                          const MotorState &state);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);
};

#include "velocity_protocol_impl.hpp"
#endif