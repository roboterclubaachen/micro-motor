#pragma once
#include "relay.hpp"

class RelayAnalyzer
{
public:
	RelayAnalyzer() = default;
	~RelayAnalyzer() = default;
	void
	setData(const std::vector<RelayUpdate>& data);

	size_t
	findSteadyStateInner(uint64_t period);

	size_t
	findLastLowInner(uint64_t period);

	float
	computeStaticGainInner(uint64_t period);

	std::pair<float, float>
	computeDeadTimeAndTimeConstantInner(uint64_t period);

	float
	computeWindowAverageOuter(size_t i, size_t windowSize);

	std::vector<size_t>
	findPeaksOuter();

	float
	findUltimateFreqOuter();

private:
	static std::vector<std::pair<size_t, size_t>>
	getPeriodInfo(const std::vector<RelayUpdate>& data);

	std::vector<std::pair<size_t, size_t>> periodData;
	std::vector<RelayUpdate> data;
};