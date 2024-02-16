#include "relay.hpp"
#include <type_traits>
#include <modm/debug/logger.hpp>
#include <string>
#include "csv_writer.hpp"

Relay::Relay()
{
	data.reserve(reserveVectorSize);
}

void
Relay::update(modm::Clock::time_point now)
{
	if (error)
	{
		// Only output 0 if we missed an update
		targetCurrent = 0.0f;
		return;
	}

	if (wasUpdated == false)
	{
		// Initialize on first update
		firstUpdate = now;
		lastUpdate = now;
		wasUpdated = true;
		currentCount = 0;
		targetCurrent = 0.0f;
		return;
	}

	if (now - lastUpdate > halfPeriod)
	{
		MODM_LOG_ERROR << "Detected time skip in relay program!" << modm::endl;
		error = true;
		targetCurrent = 0.0f;
		return;
	}

	auto timeSinceStart = now - firstUpdate;

	auto periodsSinceStart =
		((double)std::chrono::duration_cast<modm::chrono::milli_clock::duration, uint32_t>(
			 timeSinceStart)
			 .count()) /
		(((double)std::chrono::duration_cast<modm::chrono::milli_clock::duration, uint32_t>(
			  halfPeriod)
			  .count()) *
		 2);

	currentCount = std::floor(periodsSinceStart);
	if (currentCount >= count)
	{
		// We are done
		targetCurrent = 0.0f;
		return;
	}

	auto currentPointInPeriod = periodsSinceStart - currentCount;
	if (currentPointInPeriod < 0.5f)
	{
		// First half of period
		targetCurrent = 0.0f;
	} else
	{
		// Second half of period
		targetCurrent = onCurrent;
	}

	data.emplace_back(RelayUpdate{.period = currentCount,
								  .current = actualCurrent,
								  .velocity = actualVelocity,
								  .demand = targetCurrent,
								  .time = now});

	lastUpdate = now;
}

double
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

void
Relay::setValues(double actualCurrent, double actualVelocity)
{
	this->actualCurrent = actualCurrent;
	this->actualVelocity = actualVelocity;
}

void
Relay::dumpToCSV() const
{
	CSVWriter writer({"Time", "Period", "CurrentDemand", "CurrentActual", "CurrentVelocity"});
	if (!writer.create("relay.csv"))
	{
		MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
		return;
	}
	for (size_t i = 0; i < data.size(); i++)
	{
		writer.addRow({std::to_string(data[i].time.time_since_epoch().count()),
					   std::to_string(data[i].period), std::to_string(data[i].demand),
					   std::to_string(data[i].current), std::to_string(data[i].velocity)});
	}
	writer.close();
}

const std::vector<RelayUpdate>&
Relay::getData() const
{
	return data;
}
