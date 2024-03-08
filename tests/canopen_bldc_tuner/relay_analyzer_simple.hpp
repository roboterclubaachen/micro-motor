#pragma once
#include "relay_analyzer_base.hpp"
#include "pid.hpp"
#include <optional>

class RelayAnalyzerSimple : public RelayAnalyzerBase
{
private:
	static size_t
	findDemandStart(const std::vector<double>& demand, Period period);

	static double
	findZeroOffset(const std::vector<double>& values, const std::vector<double>& demands);

	static double
	computeZeroNoise(const std::vector<double>& values, const std::vector<double>& demands,
					 double zeroOffset);

	static size_t
	findFirstReactionPoint(const std::vector<double>& values, double zeroOffset, double noise);

	static double
	computeWindowAverage(const std::vector<double>& values, size_t i, size_t windowSize);

	static std::vector<size_t>
	findPeaks(const std::vector<double>& values);

	struct Line
	{
		double slope;
		double offsetY;
	};

	static Line
	computeBestFitLine(const std::vector<double>& value,
					   const std::vector<modm::Clock::time_point>& time, size_t start, size_t end);

	static double
	computeLineZeroCrossingTime(const Line& line);

	static double
	computeStaticGain(const std::vector<double>& values, const std::vector<double>& demand,
					  double steadyStateMag, size_t lowSampleIndex);

	static double
	getSteadyStateMagnitude(const std::vector<double>& values, size_t highSampleIndex,
							size_t periodEnd);

	static double
	computeTimeConstant(const Line& slope, double lowValue, double highValue);

	const std::vector<double>&
	getVectorForMode() const;

public:
	enum class AnalysisMode
	{
		Current,
		Velocity,
		Position
	};

	RelayAnalyzerSimple(AnalysisMode mode);
	virtual ~RelayAnalyzerSimple() = default;

	bool
	calc() override;

	std::optional<PID>
	getResult() const;

private:
	std::optional<PID> res{};

	const AnalysisMode mode;
};