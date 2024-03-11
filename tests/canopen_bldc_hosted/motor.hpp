#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <modm/processing/timer.hpp>

#include <librobots2/motor-sim/motor_simulation.hpp>
#include <librobots2/motor-sim/motor_bridge.hpp>
#include <librobots2/motor-sim/current_limit.hpp>
#include <micro-motor/canopen/canopen.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <librobots2/motor/bldc_motor_block_commutation.hpp>

using namespace std::literals;

using librobots2::motor_sim::CurrentLimit;
using librobots2::motor_sim::MotorBridge;
using librobots2::motor_sim::MotorSimulation;

class Motor
{
private:
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t actualPosition_{};
	modm::PeriodicTimer controlTimer_{controlTiming_};
	modm::PeriodicTimer simTimer_{100us};

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

	void
	testUpdate(int16_t pwm);

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
		MotorState0::setUnorientedCurrent(MotorSimulation::maxCurrent());
		MotorState0::setActualPosition(actualPosition_);
		MotorControl0::update<CanOpen::Device, MessageCallback>(std::forward<MessageCallback>(cb));
		if (!MotorState0::enableMotor_)
		{
			motor_.disable();
		} else
		{
			motor_.setSetpoint(MotorState0::outputPWM());
			CurrentLimit::set(MotorState0::currentLimit());
		}
		updated = true;
	}
	if (simTimer_.execute())
	{
		auto timestep = std::chrono::duration<double>(now - lastUpdate_).count();
		motor_.update();
		MotorSimulation::update(timestep);
		lastUpdate_ = now;
	}
	return updated;
}

constexpr uint8_t motor0CommutationOffset{4};
inline Motor Motor0{motor0CommutationOffset};
