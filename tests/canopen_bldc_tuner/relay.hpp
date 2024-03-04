#pragma once
#include "relay_update.hpp"

class Relay
{
private:
	constexpr static uint64_t count = 8;  // 8 Periods to run
	constexpr static modm::Clock::duration halfPeriod =
		std::chrono::duration<uint32_t, std::milli>(800);   // 1s
	constexpr static double onCurrent = 1.0f;               // 1A
	constexpr static uint64_t reserveVectorSize = 1024;

public:
	Relay();
	~Relay() = default;

	void
	update(modm::Clock::time_point now);

	double
	getDemand() const;

	void
	setValues(double actualCurrent, double actualVelocity, double actualPosition);

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
	double actualDemand = 0.0f, actualCurrent = 0.0f, actualVelocity = 0.0f, actualPosition = 0.0f;

	std::vector<RelayUpdate> data;
};
