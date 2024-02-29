#pragma once
#include <modm/processing/timer.hpp>
#include <cstdint>
#include <vector>

struct RelayUpdate
{
	uint64_t period;
	double current;
	double velocity;
	double demand;
	modm::Clock::time_point time;
};

class Relay
{
private:
	constexpr static uint64_t count = 16;  // 8 Periods to run
	constexpr static modm::Clock::duration halfPeriod =
		std::chrono::duration<uint32_t, std::milli>(200);   // 1s
	constexpr static double onCurrent = 1.0f;               // 1A
	constexpr static uint64_t reserveVectorSize = 1024;

public:
	Relay();
	~Relay() = default;

	void
	update(modm::Clock::time_point now);

	double
	getTargetCurrent() const;

	void
	setValues(double actualCurrent, double actualVelocity);

	bool
	done() const;

	bool
	errored() const;

	void
	dumpToCSV() const;

	const std::vector<RelayUpdate>&
	getData() const;

private:
	bool wasUpdated = false, error = false;
	modm::Clock::time_point firstUpdate, lastUpdate;
	uint64_t currentCount = 0;
	double targetCurrent = 0.0f, actualCurrent = 0.0f, actualVelocity = 0.0f;

	std::vector<RelayUpdate> data;
};
