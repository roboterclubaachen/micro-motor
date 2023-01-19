#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

#include "sim_motor.hpp"

using namespace std::literals;

class Motor
{
private:
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t actualPosition_{};

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

	bool
	update();
};

constexpr uint8_t motor0CommutationOffset{1};
inline Motor Motor0{motor0CommutationOffset};
