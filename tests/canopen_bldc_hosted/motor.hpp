#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <modm/processing/timer.hpp>

#include "sim/motor_bridge.hpp"
#include "sim/sim_motor.hpp"
#include <micro-motor/canopen/canopen.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <librobots2/motor/bldc_motor_block_commutation.hpp>

using namespace std::literals;

using sim::MotorBridge;
using sim::MotorSimulation;

class Motor
{
private:
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t actualPosition_{};
	modm::PeriodicTimer controlTimer_{1ms};

	modm::PreciseClock::time_point lastUpdate_{modm::PreciseClock::now()};

	librobots2::motor::BldcMotorBlockCommutation<MotorBridge> motor_;
	using Hall = librobots2::motor::HallPermutations<MotorBridge::HallPort>;
	uint_fast8_t
	readHall()
	{
		const auto& HallStates = librobots2::motor::block_commutation::SequenceLut;
		return HallStates[Hall::read(commutationOffset_) & 0b111];
	}

	void
	updatePosition();

public:
	Motor(uint_fast8_t commutationOffset);
	void
	initializeHall()
	{
		lastHallState_ = readHall();
	}

	template<typename MessageCallback>
	bool
	update(MessageCallback&& cb);

	modm::PreciseClock::time_point
	lastUpdateTime() const;
};

template<typename MessageCallback>
bool
Motor::update(MessageCallback&& cb)
{
	auto now = modm::PreciseClock::now();
	bool updated = false;
	updatePosition();
	if (controlTimer_.execute())
	{
		MotorControl0::setUnorientedCurrent(MotorSimulation::maxCurrent());
		MotorControl0::setActualPosition(actualPosition_);
		MotorControl0::update<CanOpen::Device, MessageCallback>(std::forward<MessageCallback>(cb));
		if (!MotorControl0::state().enableMotor_)
		{
			motor_.disable();
		} else
		{
			motor_.setSetpoint(MotorControl0::outputPWM());
		}
		updated = true;
	}
	auto timestep = std::chrono::duration<double>(now - lastUpdate_).count();
	motor_.update();
	MotorSimulation::update(timestep);
	lastUpdate_ = now;
	return updated;
}

constexpr uint8_t motor0CommutationOffset{0};
inline Motor Motor0{motor0CommutationOffset};
