#include "relay.hpp"
#include <type_traits>
#include <modm/debug/logger.hpp>

void
Relay::update(modm::Clock::time_point now)
{
	if (error)
	{
		targetCurrent = 0.0f;
		return;
	}

	if (wasUpdated == false)
	{
		firstUpdate = now;
		lastUpdate = now;
		wasUpdated = true;
		currentCount = 0;
		targetCurrent = 0.0f;
		return;
	}

	if (now - lastUpdate > halfPeriod)
	{
		MODM_LOG_ERROR << "Detected way to big skip in relay program!" << modm::endl;
		error = true;
		targetCurrent = 0.0f;
		return;
	}

	auto timeSinceStart = now - firstUpdate;

	auto periodsSinceStart =
		((float)std::chrono::duration_cast<modm::chrono::milli_clock::duration, uint32_t>(
			 timeSinceStart)
			 .count()) /
		(((float)std::chrono::duration_cast<modm::chrono::milli_clock::duration, uint32_t>(
			  halfPeriod)
			  .count()) *
		 2);

	currentCount = std::floor(periodsSinceStart);
	if (currentCount >= count)
	{
		targetCurrent = 0.0f;
		return;
	}

	auto currentPointInPeriod = periodsSinceStart - currentCount;
	if (currentPointInPeriod < 0.5f)
	{
		targetCurrent = 0.0f;
	} else
	{
		targetCurrent = onCurrent;
	}

	lastUpdate = now;
}

float
Relay::getTargetCurrent() const
{
	return targetCurrent;
}

bool
Relay::done() const
{
	return currentCount >= count;
}

bool
Relay::errored() const
{
	return error;
}