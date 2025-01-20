#pragma once
#include <modm/processing/timer.hpp>
#include <cstdint>
#include <vector>

struct RelayUpdate
{
	uint64_t period;
	double current;
	double velocity;
	double position;
	double demand;
	modm::Clock::time_point time;
};
