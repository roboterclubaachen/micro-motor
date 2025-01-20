#include "relay_analyzer_simple.hpp"
#include <modm/debug/logger.hpp>

double
RelayAnalyzerSimple::findZeroOffset(const std::vector<double>& values,
									const std::vector<double>& demands)
{
	MODM_LOG_INFO << "Computing zero offset..." << modm::endl;
	double accumulator = 0.0;
	size_t count = 0;
	for (size_t i = 0; i < demands.size(); i++)
	{
		if (demands[i] != 0.0) break;
		accumulator += values[i];
		count++;
	}
	accumulator /= count;
	MODM_LOG_INFO << "Averaged the first " << count << " samples:" << accumulator << modm::endl;
	return accumulator;
}

double
RelayAnalyzerSimple::computeZeroNoise(const std::vector<double>& values,
									  const std::vector<double>& demands, double zeroOffset)
{
	MODM_LOG_INFO << "Computing zero noise..." << modm::endl;
	double max = values[0], min = values[0];
	size_t count = 0;
	for (size_t i = 0; i < demands.size(); i++)
	{
		if (demands[i] != 0.0) break;
		if (max < values[i]) max = values[i];
		if (min > values[i]) min = values[i];
		count++;
	}
	MODM_LOG_INFO << "Analyzed the first " << count << " samples: [" << max << ":" << min << "]"
				  << modm::endl;
	return std::max(std::abs(max - zeroOffset), std::abs(zeroOffset - min));
}

size_t
RelayAnalyzerSimple::findFirstReactionPoint(const std::vector<double>& values, double zeroOffset,
											double noise)
{
	const auto sigma = 1.1f;
	size_t i = 0;
	for (; i < values.size(); i++)
	{
		if (values[i] > zeroOffset + noise * sigma) break;
	}
	return i - 1;
}

double
RelayAnalyzerSimple::computeWindowAverage(const std::vector<double>& values, size_t i,
										  size_t windowSize)
{
	auto hWS = windowSize / 2;
	if (hWS * 2 < windowSize) hWS++;
	double windowAvg = 0.0f;
	for (size_t j = i - hWS; j < i - hWS + windowSize; j++) { windowAvg += values[j]; }
	return windowAvg / windowSize;
}

std::vector<size_t>
RelayAnalyzerSimple::findPeaks(const std::vector<double>& values)
{
	std::vector<size_t> out;
	constexpr int windowSize = 5;
	constexpr int halfWindowSize = std::ceil(windowSize / 2.0f);

	if (values.size() < windowSize) return {};
	std::vector<double> averages;
	averages.reserve(values.size());
	for (size_t i = halfWindowSize; i < values.size() - halfWindowSize; i++)
	{
		averages.push_back(computeWindowAverage(values, i, windowSize));
	}
	constexpr size_t stride = 1;
	// TODO derive from period length
	constexpr size_t localMaxWindow = 400;

	if (averages.size() < stride * 2 + 1) return {};
	bool plateaud = false;
	size_t plateauStartIndex = 0;
	double plateauStartValue = 0.0;
	for (size_t i = stride; i < averages.size() - stride; i++)
	{
		double pastMax{0.0}, futureMax{0.0};
		const auto pastMaxWindowStart = (i > localMaxWindow) ? i - localMaxWindow : 0;
		for (size_t j = i - 1; j > pastMaxWindowStart; j--)
		{
			if (averages[j] > pastMax) pastMax = averages[j];
			if (j == 0) break;
		}
		for (size_t j = i + 1; j < i + localMaxWindow && j < averages.size(); j++)
		{
			if (averages[j] > futureMax) futureMax = averages[j];
		}

		auto curr = averages[i];
		if (pastMax < curr)
		{
			plateauStartIndex = i;
			plateauStartValue = curr;
			plateaud = true;
		}

		if (plateauStartValue > futureMax)
		{
			if (plateaud) { out.push_back(plateauStartIndex + halfWindowSize); }
			plateaud = false;
		}
	}
	return out;
}

RelayAnalyzerSimple::Line
RelayAnalyzerSimple::computeBestFitLine(const std::vector<double>& value,
										const std::vector<modm::Clock::time_point>& time,
										size_t start, size_t end)
{
	const auto raw_count = end - start;
	const auto holdOffL = (size_t)(raw_count * 0.1);
	const auto holdOffR = (size_t)(raw_count * 0.3);
	const auto count = end - start - holdOffL - holdOffR;
	MODM_LOG_INFO << "Holding " << holdOffL << "," << holdOffR << " samples to compute line."
				  << modm::endl;
	const auto startTime =
		std::chrono::duration_cast<std::chrono::duration<double>>(time[start + holdOffL] - time[0])
			.count();
	const auto endTime =
		std::chrono::duration_cast<std::chrono::duration<double>>(time[end - holdOffR] - time[0])
			.count();
	MODM_LOG_INFO << "Computing best fit line from " << startTime << "s to " << endTime << "s..."
				  << modm::endl;
	double sumY = 0.0, sumX = 0.0;
	double sumX2 = 0.0, sumXY = 0.0;
	for (size_t i = start + holdOffL; i < end - holdOffR; i++)
	{
		const auto valX =
			std::chrono::duration_cast<std::chrono::duration<double>>(time[i] - time[0]).count();
		sumY += value[i];
		sumX += valX;
		sumX2 += valX * valX;
		sumXY += valX * value[i];
	}
	const auto meanX = sumX / count;
	const auto meanY = sumY / count;

	const auto slope = (sumXY - sumX * meanY) / (sumX2 - sumX * meanX);
	const auto yInt = meanY - slope * meanX;
	MODM_LOG_INFO << "Line: " << slope << " * x + " << yInt << modm::endl;
	return Line{.slope = slope, .offsetY = yInt};
}

double
RelayAnalyzerSimple::computeLineZeroCrossingTime(const Line& line)
{
	return (-line.offsetY) / line.slope;
}

double
RelayAnalyzerSimple::computeStaticGain(const std::vector<double>& values,
									   const std::vector<double>& demand, double steadyStateMag,
									   size_t lowSampleIndex)
{
	if (lowSampleIndex >= values.size()) { return 0.0f; }

	auto valueHigh = steadyStateMag;
	auto valueLow = values[lowSampleIndex];
	auto demandHigh = demand[lowSampleIndex + 1];  // Hacky
	auto demandLow = demand[lowSampleIndex];

	auto demandDiff = demandHigh - demandLow;
	auto valueDiff = valueHigh - valueLow;

	return valueDiff / demandDiff;
}

size_t
RelayAnalyzerSimple::findDemandStart(const std::vector<double>& demand, Period period)
{
	auto start = period.first;
	auto end = start + period.second;

	auto high = demand[end - 1];
	for (size_t i = end - 1; start <= i; i--)
	{
		if (demand[i] < high) return i;
		if (i == 0) break;  // Prevent underflow
	}
	return demand.size();
}

double
RelayAnalyzerSimple::getSteadyStateMagnitude(const std::vector<double>& values,
											 size_t highSampleIndex, size_t periodEnd)
{
	double avg = 0.0;
	for (size_t i = highSampleIndex; i < periodEnd; i++) { avg += values[i]; }
	return avg / (periodEnd - highSampleIndex);
}

RelayAnalyzerSimple::RelayAnalyzerSimple(AnalysisMode mode) : mode(mode) {}

const std::vector<double>&
RelayAnalyzerSimple::getVectorForMode() const
{
	switch (mode)
	{
		case AnalysisMode::Current:
			return currents;
		case AnalysisMode::Velocity:
			return velocities;
		case AnalysisMode::Position:
			return positions;
		default:
			abort();
	}
}

double
RelayAnalyzerSimple::computeTimeConstant(const Line& line, double lowValue, double highValue)
{
	// slope*dt+low = 0.632*high
	return (0.632 * highValue - lowValue) / line.slope;
}

bool
RelayAnalyzerSimple::calc()
{
	res = std::nullopt;

	const std::vector<double>& values = getVectorForMode();

	const auto zeroOffset = findZeroOffset(values, demands);
	const auto noise = computeZeroNoise(values, demands, zeroOffset);
	const auto firstReactionPoint = findFirstReactionPoint(values, zeroOffset, noise);
	const auto peaks = findPeaks(values);

	analysis.cur_peaks = peaks;
	analysis.cur_valleys.push_back(firstReactionPoint);

	const auto line = computeBestFitLine(values, times, firstReactionPoint, peaks[0]);

	const auto zeroCrossingTime = computeLineZeroCrossingTime(line);

	const auto demandStart = findDemandStart(demands, periodData[0]);
	const auto demandStartTime =
		std::chrono::duration_cast<std::chrono::duration<double>>(times[demandStart] - times[0])
			.count();
	MODM_LOG_INFO << "Demand starts at " << demandStartTime << modm::endl;
	MODM_LOG_INFO << "ZeroCrossing is at " << zeroCrossingTime << modm::endl;

	if (demandStartTime >= zeroCrossingTime) return false;
	const auto deadTime = zeroCrossingTime - demandStartTime;

	const auto steadyStateMag =
		getSteadyStateMagnitude(values, peaks[0], periodData[0].first + periodData[0].second);

	const auto staticGain = computeStaticGain(values, demands, steadyStateMag, demandStart);
	const auto timeConstant = computeTimeConstant(line, values[firstReactionPoint], steadyStateMag);

	MODM_LOG_INFO << "L_1 " << deadTime << " K_1 " << staticGain << " T_1 " << timeConstant
				  << modm::endl;

	res = PID{
		.p = 0.7 * timeConstant / (staticGain * deadTime),
		.i = 0.304 * timeConstant / (staticGain * deadTime),
		.d = 0.0,
	};

	return true;
}

std::optional<PID>
RelayAnalyzerSimple::getResult() const
{
	return res;
}