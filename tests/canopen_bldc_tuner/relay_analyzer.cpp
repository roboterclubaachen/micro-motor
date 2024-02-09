#include "relay_analyzer.hpp"
#include <Eigen/Dense>

using namespace std::literals;

/// Return index,length pairs for periods in data
std::vector<std::pair<size_t, size_t>>
RelayAnalyzer::getPeriodInfo(const std::vector<RelayUpdate>& data)
{
	std::vector<std::pair<size_t, size_t>> out;
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
		}
	}
	out.push_back({index, size});
	return out;
}

void
RelayAnalyzer::setData(const std::vector<RelayUpdate>& data)
{
	this->data = data;
	this->periodData = getPeriodInfo(data);
}

size_t
RelayAnalyzer::findSteadyStateInner(uint64_t period)
{
	constexpr float sigma = 1.1f;
	if (period > periodData.size()) { return data.size(); }

	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	if (interval.second < 2) { return data.size(); }

	std::vector<float> diff;
	diff.reserve(interval.second - 1);

	// Calculate percentage difference between samples
	for (size_t i = start + 1; i < end; i++)
	{
		diff.push_back((data[i].current - data[i - 1].current) / data[i - 1].current);
	}

	uint64_t minIndex = diff.size() - 1;
	float min = diff[minIndex];

	// Iterate backwards through samples and stop if we go outside our sigma range
	for (size_t i = 2; i <= diff.size(); i++)
	{
		auto index = diff.size() - i;
		if (std::abs(diff[index]) < min * sigma)
		{
			min = diff[index];
			minIndex = index;
		}
	}

	return start + minIndex - 1;  // Convert back to index in data
}

size_t
RelayAnalyzer::findLastLowInner(uint64_t period)
{
	if (period > periodData.size()) { return data.size(); }
	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	auto high = data[end - 1].demand;
	for (size_t i = end - 1; start <= end; i--)
	{
		if (data[i].demand < high) return i;
		if (i == 0) break;  // Prevent underflow
	}
	return data.size();
}

float
RelayAnalyzer::computeStaticGainInner(uint64_t period)
{
	if (period > periodData.size()) { return 0.0f; }

	auto highSample = findSteadyStateInner(period);
	if (highSample >= data.size()) { return 0.0f; }

	auto lowSample = findLastLowInner(period);
	if (lowSample >= data.size()) { return 0.0f; }

	auto currentHigh = data[highSample].current;
	auto currentLow = data[lowSample].current;
	auto demandHigh = data[highSample].demand;
	auto demandLow = data[lowSample].demand;

	auto demandDiff = demandHigh - demandLow;
	auto currentDiff = currentHigh - currentLow;

	return currentDiff / demandDiff;
}

std::pair<float, float>
RelayAnalyzer::computeDeadTimeAndTimeConstantInner(uint64_t period)
{
	auto k2 = computeStaticGainInner(period);
	if (k2 == 0.0f) return {};

	auto timeBase = data[0].time;
	auto interval = periodData[period];
	auto start = interval.first;

	using MatrixX2d = Eigen::Matrix<float, Eigen::Dynamic, 2>;
	using VectorXd = Eigen::Matrix<float, Eigen::Dynamic, 1>;

	MatrixX2d psi = MatrixX2d(interval.second, 2);
	VectorXd gamma = VectorXd(interval.second, 1);

	float a = 0.0f;

	for (size_t i = 0; i < interval.second; i++)
	{
		auto current = data[start + i].current;
		auto timeSinceStart = data[start + i].time - timeBase;
		a += timeSinceStart.count() * current;

		psi(i, 0) = current;
		psi(i, 1) = k2;

		gamma(i, 0) = k2 * timeSinceStart.count() - a;
	}

	auto XStar = (psi.transpose() * psi).inverse() * psi.transpose() * gamma;

	return {XStar(0, 0), XStar(0, 1)};
}

float
RelayAnalyzer::computeWindowAverageOuter(size_t i, size_t windowSize)
{
	auto hWS = windowSize / 2;
	if (hWS * 2 < windowSize) hWS++;
	float windowAvg = 0.0f;
	for (size_t j = i - hWS; j < i - hWS + windowSize; j++) { windowAvg += data[j].velocity; }
	return windowAvg / windowSize;
}

std::vector<size_t>
RelayAnalyzer::findPeaksOuter()
{
	std::vector<size_t> out;
	constexpr int windowSize = 5;
	constexpr int halfWindowSize = 3;
	if (data.size() < windowSize) return {};
	std::vector<float> averages;
	averages.reserve(data.size());
	for (size_t i = halfWindowSize; i < data.size() - halfWindowSize; i++)
	{
		averages.push_back(computeWindowAverageOuter(i, windowSize));
	}
	if (averages.size() < 3) return {};

	for (size_t i = 1; i < averages.size() - 1; i++)
	{
		auto prev = averages[i - 1];
		auto curr = averages[i];
		auto next = averages[i + 1];
		if (prev < curr && curr > next) { out.push_back(i + halfWindowSize); }
	}
	return out;
}

float
RelayAnalyzer::findUltimateFreqOuter()
{
	auto peaks = findPeaksOuter();
	if (peaks.size() == 0) return 0.0f;

	size_t count = 0;
	modm::Clock::duration acc(0);
	auto lastTime = data[peaks[0]].time;
	for (size_t i = 1; i < peaks.size(); i++)
	{
		auto currTime = data[peaks[i]].time;
		auto diff = currTime - lastTime;
		lastTime = currTime;
		acc += diff;
		count++;
	}
	acc /= count;
	return 1000.0f / (float)acc.count();
}