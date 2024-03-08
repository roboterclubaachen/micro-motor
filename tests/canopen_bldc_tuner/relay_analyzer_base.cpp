#include "relay_analyzer_base.hpp"
#include "csv_writer.hpp"
#include <modm/debug/logger.hpp>

void
RelayAnalyzerBase::setData(const std::vector<RelayUpdate>& data)
{
	MODM_LOG_INFO << "Sample count: " << data.size() << modm::endl;
	for (size_t i = 0; i < data.size(); i++)
	{
		currents.push_back(data[i].current);
		demands.push_back(data[i].demand);
		velocities.push_back(data[i].velocity);
		positions.push_back(data[i].position);
		times.push_back(data[i].time);
	}
	periodData = getPeriodInfo(data);
	MODM_LOG_INFO << "Period count: " << periodData.size() << modm::endl;
}

/// Return index,length pairs for periods in data
std::vector<RelayAnalyzerBase::Period>
RelayAnalyzerBase::getPeriodInfo(const std::vector<RelayUpdate>& data)
{
	std::vector<Period> out;
	size_t currentPeriod = 0, index = 0, size = 0;
	for (size_t i = 0; i < data.size(); i++)
	{
		auto& update = data[i];
		if (update.period == currentPeriod) { size++; }
		if (update.period > currentPeriod)
		{
			out.push_back({index, size});
			index = i;
			size = 1;
			currentPeriod++;
		}
	}
	out.push_back({index, size});
	return out;
}

void
RelayAnalyzerBase::dumpVector(const std::string& name, const std::vector<size_t>& samples) const
{
	CSVWriter writer(
		{"Time", "CurrentDemand", "CurrentActual", "CurrentVelocity", "CurrentPosition"});
	if (!writer.create(name))
	{
		MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
		return;
	}
	for (size_t i = 0; i < samples.size(); i++)
	{
		auto sample = samples[i];
		writer.addRow({std::to_string((times[sample] - times[0]).count()),
					   std::to_string(demands[sample]), std::to_string(currents[sample]),
					   std::to_string(velocities[sample]), std::to_string(positions[sample])});
	}
	writer.close();
}

void
RelayAnalyzerBase::dumpToCSV() const
{
	dumpVector("relay_vel_peaks.csv", analysis.vel_peaks);
	dumpVector("relay_vel_valleys.csv", analysis.vel_valleys);
	dumpVector("relay_cur_peaks.csv", analysis.cur_peaks);
	dumpVector("relay_cur_valleys.csv", analysis.cur_valleys);
	dumpVector("relay_pos_peaks.csv", analysis.pos_peaks);
	dumpVector("relay_pos_valleys.csv", analysis.pos_valleys);
}
