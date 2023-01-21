#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include <librobots2/motor/bldc_motor_block_commutation.hpp>
#include <micro-motor/hardware.hpp>
#include <micro-motor/canopen/motor_control.hpp>
#include <modm/processing/timer.hpp>

using namespace std::literals;

class Motor
{
private:
	int32_t actualPosition_{};
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	modm::PeriodicTimer controlTimer_{10ms};

	librobots2::motor::BldcMotorBlockCommutation<Board::Motor> motor_;
	using Hall = librobots2::motor::HallPermutations<Board::Motor::HallPort>;
	uint_fast8_t
	readHall()
	{
		const auto &HallStates = librobots2::motor::block_commutation::SequenceLut;
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

	bool
	update();
};

constexpr uint8_t motor0CommutationOffset{1};
inline Motor Motor0{motor0CommutationOffset};
