#pragma once
#include "relay_update.hpp"

struct RelayConfig
{
	uint64_t periodCount = 8;
	modm::Clock::duration onPeriod = std::chrono::duration<uint32_t, std::milli>(400);
	modm::Clock::duration offPeriod = std::chrono::duration<uint32_t, std::milli>(400);
	double onValue = 1.0;
};

class Relay
{
private:
	constexpr static uint64_t reserveVectorSize = 1024;

public:
	Relay(const RelayConfig& config);
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
	const RelayConfig config;

	bool wasUpdated = false, error = false;
	modm::Clock::time_point firstUpdate, lastUpdate;
	uint64_t currentCount = 0;
	double actualDemand = 0.0f, actualCurrent = 0.0f, actualVelocity = 0.0f, actualPosition = 0.0f;

	std::vector<RelayUpdate> data;
};
