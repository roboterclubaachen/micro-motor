#pragma once
#include "relay.hpp"
#include <complex>
#include <span>

class RelayAnalyzer
{
public:
	RelayAnalyzer() = default;
	~RelayAnalyzer() = default;
	void
	setData(const std::vector<RelayUpdate>& data);

	size_t
	findSteadyStateInner(uint64_t period) const;

	size_t
	findLastLowInner(uint64_t period) const;

	double
	computeStaticGainInner(size_t highSampleIndex, size_t lowSampleIndex) const;

	std::pair<double, double>
	computeDeadTimeAndTimeConstantInner(uint64_t period, double static_gain) const;

	double
	computeDeadTimeWhole(uint64_t period) const;

	double
	computeWindowAverageOuter(size_t i, size_t windowSize) const;

	std::vector<size_t>
	findPeaksOuter() const;

	uint64_t
	findStaticOscillationSampleOuter(const std::vector<size_t>& peaks) const;

	uint64_t
	findPeriodForSample(size_t sample) const;

	// Actually Omega
	double
	findUltimateFreqOuter(const std::vector<size_t>& peaks) const;  // omega_u

	std::complex<double>
	getFreqResponseWhole(uint64_t static_oscillation_period, double ultimate_freq) const;  // G_p

	std::complex<double>
	getFreqResponseInner(double k_2, double l_2, double t_2, double omega_u) const;  // G_p2

	double
	getDeadTimeOuter(uint64_t period) const;

	void
	calc() const;

private:
	static std::vector<std::pair<size_t, size_t>>
	getPeriodInfo(const std::vector<RelayUpdate>& data);

	std::vector<std::pair<size_t, size_t>> periodData;
	std::vector<RelayUpdate> data;
};