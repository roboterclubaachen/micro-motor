#pragma once
#include <modm/processing/timer.hpp>
#include <cstdint>

class Relay
{
private:
	constexpr static uint64_t count = 128;  // 128 Periods to run
	constexpr static modm::Clock::duration halfPeriod =
		std::chrono::duration<uint32_t, std::milli>(1000);  // 1s
	constexpr static float onCurrent = 1.0f;                // 1A

public:
	Relay() = default;
	~Relay() = default;

	void
	update(modm::Clock::time_point now);

	float
	getTargetCurrent() const;

	bool
	done() const;

	bool
	errored() const;

private:
	bool wasUpdated = false, error = false;
	modm::Clock::time_point firstUpdate, lastUpdate;
	uint64_t currentCount = 0;
	float targetCurrent = 0.0f;
};
