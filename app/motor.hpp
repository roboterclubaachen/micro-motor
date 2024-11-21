#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include <librobots2/motor/bldc_motor_block_commutation.hpp>
#include <librobots2/motor/bldc_motor_current.hpp>
#include <micro-motor/hardware.hpp>
#include <micro-motor/micro-motor.hpp>
#include <micro-motor/canopen/canopen.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <modm/processing/timer.hpp>

using namespace std::literals;

constexpr MotorInfo chinesium = {
	.winding_r_ohm = 1.8f,
	.hall_offset = 1,
};

constexpr MotorInfo maxon = {
	.winding_r_ohm = 3.8f,
	.hall_offset = 0,
};

constexpr MotorInfo watney = {
	.winding_r_ohm = 3.8f,
	.hall_offset = 1,
};

using MotorState0 = MotorState<0>;
using MotorControl0 = MotorControl_t<0, Board::Encoder>;
using CanOpen =
		modm_canopen::CanopenDevice<modm_canopen::generated::micromotor_OD, MotorControl0>;

class Motor
{
private:
	int32_t actualPosition_{};
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};

	modm::PeriodicTimer controlTimer_{controlTiming_};
	librobots2::motor::BldcMotorCurrent<16> current_;
	modm::filter::MovingAverage<float, 128> max_current_;

	librobots2::motor::BldcMotorBlockCommutation<Board::Motor> motor_;
	using Hall = librobots2::motor::HallPermutations<Board::Motor::HallPort>;
	uint_fast8_t
	readHall()
	{
		const auto& HallStates = librobots2::motor::block_commutation::SequenceLut;
		return HallStates[Hall::read(commutationOffset_) & 0b111];
	}

	void
	updateCurrent();

	void
	updatePosition();

public:
	Motor(MotorInfo motorInfo);
	void
	initializeHall()
	{
		lastHallState_ = readHall();
	}

	void
	updateMotor();

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
	updateCurrent();
	if (controlTimer_.execute())
	{
		MotorState0::setActualPosition(actualPosition_);
		// auto current = current_.getMagnitude();
		// auto velocity = MotorControl0::state().actualVelocity_.getValue();
		// if (std::signbit(velocity)) current = -current;
		MotorState0::setUnorientedCurrent(max_current_.getValue());
		MotorState0::setOrientedCurrent(current_.getOrientedCurrent());
		MotorState0::setOrientedCurrentAngleDiff(current_.getAngleDifference());
		MotorControl0::update<CanOpen, MessageCallback>(std::forward<MessageCallback>(cb));
		if (!MotorState0::enableMotor_)
		{
			motor_.disable();
			micro_motor::setCurrentLimitAmps(0);
		} else
		{
			motor_.setSetpoint(MotorState0::outputPWM());
			micro_motor::setCurrentLimitAmps(MotorState0::currentLimit());
		}
		updated = true;
	}
	return updated;
}

inline Motor Motor0{watney};
