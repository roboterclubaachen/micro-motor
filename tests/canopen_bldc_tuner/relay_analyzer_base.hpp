#pragma once
#include "relay_update.hpp"
#include <string>

class RelayAnalyzerBase
{

public:
	using Period = std::pair<size_t, size_t>;

	struct Analysis
	{
		std::vector<size_t> pos_peaks;
		std::vector<size_t> pos_valleys;

		std::vector<size_t> vel_peaks;
		std::vector<size_t> vel_valleys;

		std::vector<size_t> cur_peaks;
		std::vector<size_t> cur_valleys;
	};

	RelayAnalyzerBase() = default;
	virtual ~RelayAnalyzerBase() = default;

	void
	setData(const std::vector<RelayUpdate>& data);

	virtual bool
	calc() = 0;

	void
	dumpVector(const std::string& name, const std::vector<size_t>& samples) const;

	void
	dumpToCSV() const;

private:
	static std::vector<Period>
	getPeriodInfo(const std::vector<RelayUpdate>& data);

protected:
	mutable Analysis analysis{};
	std::vector<Period> periodData;
	std::vector<double> currents, demands, velocities, positions;
	std::vector<modm::Clock::time_point> times;
};