#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <modm/processing/timer.hpp>

#include "sim_motor.hpp"
#include <micro-motor/canopen/canopen.hpp>
#include <motor-canopen/motor_control.hpp>

using namespace std::literals;

class Motor
{
private:
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t actualPosition_{};
	modm::PeriodicTimer controlTimer_{10ms};

	MotorSimulation dummy_{};

	void
	updatePosition();

public:
	Motor(uint_fast8_t commutationOffset);
	void
	initializeHall()
	{
		lastHallState_ = dummy_.hall();
	}

	inline MotorSimulation&
	dummy()
	{
		return dummy_;
	};

	template<typename MessageCallback>
	bool
	update(MessageCallback&& cb);
};

template<typename MessageCallback>
bool
Motor::update(MessageCallback&& cb)
{
	bool updated = false;
	updatePosition();
	if (controlTimer_.execute())
	{
		MotorControl0::setActualPosition(actualPosition_);
		MotorControl0::update<CanOpen::Device, MessageCallback>(std::forward<MessageCallback>(cb));
		dummy_.setInputVoltageInt(MotorControl0::outputPWM());
		updated = true;
	}
	dummy_.update(0.1f);
	return updated;
}

constexpr uint8_t motor0CommutationOffset{1};
inline Motor Motor0{motor0CommutationOffset};
