#pragma once
#include "relay_analyzer_base.hpp"
#include <complex>
#include <span>

class RelayAnalyzer : public RelayAnalyzerBase
{
private:
	static size_t
	findSteadyState(const std::vector<double>& value, Period period, size_t lastLowSample);

	static size_t
	findDemandStart(const std::vector<double>& demand, Period period);

	static size_t
	findLastLow( const std::vector<size_t>& valleys,const std::vector<Period>& perioData,Period period) ;

	static double
	computeStaticGain(const std::vector<double>& values, const std::vector<double>& demand, size_t highSampleIndex, size_t lowSampleIndex);

	std::pair<double, double>
	computeDeadTimeAndTimeConstantInner(Period period, double static_gain,
										size_t highSampleIndex) const;

	static double
	computeDeadTime(const std::vector<double>& values, Period period);

	static double
	computeWindowAverage(const std::vector<double>& values, size_t i, size_t windowSize);


	static std::vector<size_t>
	findPeaks(const std::vector<double>& values) ;

	static std::vector<size_t>
	findValleys(const std::vector<double>& values) ;

	static uint64_t
	findStaticOscillationSample(const std::vector<size_t>& peaks, const std::vector<double>& values,
								const std::vector<modm::Clock::time_point>& time);

	static Period
	findPeriodForSample(const std::vector<Period>& periodData, size_t sample);

	// Actually Omega
	static double
	findUltimateFreqOuter(const std::vector<size_t>& peaks, const std::vector<modm::Clock::time_point>& time);  // omega_u

	static std::complex<double>
	getFreqResponseWhole(const std::vector<double>& demand, const std::vector<double>& values,
						 const std::vector<modm::Clock::time_point>& time, double ultimate_freq,
						 size_t startSample, size_t timeStartSample);  // G_p

	static std::complex<double>
	getFreqResponseInner(double k_2, double l_2, double t_2, double omega_u);  // G_p2

	static double
	getDeadTimeOuter(const std::vector<double>& demand,
					 const std::vector<modm::Clock::time_point>& time, Period period,
					 size_t lastLowSample);

public:
	RelayAnalyzer() = default;
	virtual ~RelayAnalyzer() = default;

	bool
	calc() override;
};