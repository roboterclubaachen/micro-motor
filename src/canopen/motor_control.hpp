#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#include "motor_state.hpp"
#include <modm-canopen/canopen_device.hpp>

#include "heartbeat_protocol.hpp"
#include "identity_protocol.hpp"
#include "position_protocol.hpp"
#include "pwm_protocol.hpp"
#include "quickstop_protocol.hpp"
#include "velocity_protocol.hpp"

template <size_t id, typename... Modes> class MotorControl {
private:
  static inline MotorState state_{};

  template <typename Device, typename MessageCallback, typename First,
            typename Second, typename... Rest>
  static bool updateMode(MessageCallback &&cb);

  template <typename Device, typename MessageCallback, typename First>
  static bool updateMode(MessageCallback &&cb);

public:
  static inline const MotorState &state() { return state_; }

  template <typename Device, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  static inline void setActualPosition(int32_t position) {
    state_.actualPosition_ = position;
  }

  static inline int16_t outputPWM() { return state_.outputPWM_; }

  template <typename ObjectDictionary>
  constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);
};

template <size_t id>
using MotorControl_t =
    MotorControl<id, IdentityProtocol<id>, HeartbeatProtocol<id>,
                 PWMProtocol<id>, VelocityProtocol<id>,
                 PositionProtocol<id, VelocityProtocol<id>>,
                 QuickstopProtocol<id, VelocityProtocol<id>>>;

#include "motor_control_impl.hpp"
#endif