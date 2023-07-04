#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include <librobots2/motor/bldc_motor_block_commutation.hpp>
#include <librobots2/motor/bldc_motor_current.hpp>
#include <micro-motor/hardware.hpp>
#include <micro-motor/canopen/canopen.hpp>
#include <librobots2/motor-canopen/motor_control.hpp>
#include <modm/processing/timer.hpp>

using namespace std::literals;

class Motor
{
private:
	int32_t actualPosition_{};
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	modm::PeriodicTimer controlTimer_{1ms};
	librobots2::motor::BldcMotorCurrent<1024> current_;
	modm::filter::MovingAverage<float, 1024> max_current_;

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
	Motor(uint_fast8_t commutationOffset);
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
		MotorControl0::setActualPosition(actualPosition_);
		// auto current = current_.getMagnitude();
		// auto velocity = MotorControl0::state().actualVelocity_.getValue();
		// if (std::signbit(velocity)) current = -current;
		MotorControl0::setOrientedCurrent(max_current_.getValue());
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
	return updated;
}

constexpr uint8_t motor0CommutationOffset{1};
inline Motor Motor0{motor0CommutationOffset};
