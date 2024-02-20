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
RelayAnalyzer::findSteadyStateInner(uint64_t period) const
{
	constexpr double sigma = 0.1f;
	if (period > periodData.size()) { return data.size(); }

	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	if (interval.second < 2) { return data.size(); }

	std::vector<double> diff;
	diff.reserve(interval.second - 1);

	// Calculate percentage difference between samples
	for (size_t i = start + 1; i < end; i++)
	{
		diff.push_back((data[i].current - data[i - 1].current) / data[i - 1].current);
	}

	uint64_t minIndex = diff.size() - 1;
	double min = diff[minIndex];

	// Iterate backwards through samples and stop if we go outside our sigma range
	for (size_t i = 2; i <= diff.size(); i++)
	{
		auto index = diff.size() - i;
		if (std::abs(diff[index]) < min * (1.0 + sigma))
		{
			min = diff[index];
			minIndex = index;
		}
	}

	return start + minIndex - 1;  // Convert back to index in data
}

size_t
RelayAnalyzer::findLastLowInner(uint64_t period) const
{
	if (period > periodData.size()) { return data.size(); }
	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	auto high = data[end - 1].demand;
	for (size_t i = end - 1; start <= i; i--)
	{
		if (data[i].demand < high) return i;
		if (i == 0) break;  // Prevent underflow
	}
	return data.size();
}

double
RelayAnalyzer::computeStaticGainInner(size_t highSample, size_t lowSample) const
{
	if (highSample >= data.size()) { return 0.0f; }
	if (lowSample >= data.size()) { return 0.0f; }

	auto currentHigh = data[highSample].current;
	auto currentLow = data[lowSample].current;
	auto demandHigh = data[highSample].demand;
	auto demandLow = data[lowSample].demand;

	auto demandDiff = demandHigh - demandLow;
	auto currentDiff = currentHigh - currentLow;

	return currentDiff / demandDiff;
}

std::pair<double, double>
RelayAnalyzer::computeDeadTimeAndTimeConstantInner(uint64_t period, double k2) const
{

	if (period > periodData.size()) { return {}; }

	auto timeBase = data[0].time;
	auto interval = periodData[period];
	auto start = interval.first;

	using MatrixX2d = Eigen::Matrix<double, Eigen::Dynamic, 2>;
	using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

	MatrixX2d psi = MatrixX2d(interval.second, 2);
	VectorXd gamma = VectorXd(interval.second, 1);

	double a = 0.0f;

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

double
RelayAnalyzer::computeWindowAverageOuter(size_t i, size_t windowSize) const
{
	auto hWS = windowSize / 2;
	if (hWS * 2 < windowSize) hWS++;
	double windowAvg = 0.0f;
	for (size_t j = i - hWS; j < i - hWS + windowSize; j++) { windowAvg += data[j].velocity; }
	return windowAvg / windowSize;
}

std::vector<size_t>
RelayAnalyzer::findPeaksOuter() const
{
	std::vector<size_t> out;
	constexpr int windowSize = 5;
	constexpr int halfWindowSize = 3;
	if (data.size() < windowSize) return {};
	std::vector<double> averages;
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

double
RelayAnalyzer::findUltimateFreqOuter(const std::vector<size_t>& peaks) const
{
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
	return 1000.0f * M_PI * 2 / (double)acc.count();
}

uint64_t
RelayAnalyzer::findStaticOscillationSampleOuter(const std::vector<size_t>& peaks) const
{
	constexpr double sigmaT = 0.1f, sigmaY = 0.1f;
	constexpr size_t minCountInWindow = 5;

	if (peaks.size() <= 2) return data.size();

	auto tDiff = (data[peaks[1]].time - data[peaks[0]].time).count() / 1000.0;
	auto yDiff = (data[peaks[1]].velocity - data[peaks[0]].velocity);

	size_t inWindow = 0;

	for (size_t i = 2; i < peaks.size(); i++)
	{
		const auto tDiffMax = std::max(tDiff * (1.0 + sigmaT), tDiff * (1.0 - sigmaT));
		const auto tDiffMin = std::min(tDiff * (1.0 + sigmaT), tDiff * (1.0 - sigmaT));

		const auto yDiffMax = std::max(yDiff * (1.0 + sigmaY), yDiff * (1.0 - sigmaY));
		const auto yDiffMin = std::min(yDiff * (1.0 + sigmaY), yDiff * (1.0 - sigmaY));

		auto next_tDiff = (data[peaks[i]].time - data[peaks[i - 1]].time).count() / 1000.0;
		auto next_yDiff = (data[peaks[i]].velocity - data[peaks[i - 1]].velocity);

		if (tDiffMin <= tDiff && tDiff <= tDiffMax && yDiffMin <= yDiff && yDiff <= yDiffMax)
		{
			inWindow++;
			if (inWindow >= minCountInWindow) { return i - 1; }
		} else
		{
			inWindow = 0;
		}

		tDiff = next_tDiff;
		yDiff = next_yDiff;
	}
	return data.size();
}

uint64_t
RelayAnalyzer::findPeriodForSample(size_t sample) const
{
	if (sample >= data.size()) return periodData.size();
	for (size_t i = 0; i < periodData.size(); i++)
	{
		if (periodData[i].first <= sample && sample <= periodData[i].first + periodData[i].second)
		{
			return i;
		}
	}
	return periodData.size();
}

std::complex<double>
RelayAnalyzer::getFreqResponseWhole(uint64_t period, double omega_u) const
{
	if (period > periodData.size()) { return {}; }
	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	if (end - start < 2) return {};

	auto it = data.begin() + start;
	modm::chrono::milli_clock::time_point lastTime = it++->time;

	double timeAcc = 0.0;
	std::complex<double> inAcc = {}, outAcc = {};
	const auto t_u = 2.0 * M_PI / omega_u;
	while (timeAcc < t_u && it != data.end())
	{
		const auto t = it->time.time_since_epoch().count() / 1000.0;
		const auto timeDiff = (double)(it->time - lastTime).count() / 1000.0;
		timeAcc += timeDiff;
		std::complex<double> exponent{};
		exponent.real(0.0);
		exponent.imag(-omega_u * t);
		inAcc += it->demand * timeDiff * std::exp(exponent);
		outAcc += it->velocity * timeDiff * std::exp(exponent);
		lastTime = it++->time;
	}
	if (it == data.end()) { return {}; }

	return (inAcc) / (outAcc);
}

std::complex<double>
RelayAnalyzer::getFreqResponseInner(double k_2, double l_2, double t_2, double omega_u) const
{
	return k_2 / (std::complex<double>{1, t_2 * omega_u}) *
		   std::exp(std::complex<double>{0, -l_2 * omega_u});
}

double
RelayAnalyzer::getDeadTimeOuter(uint64_t period) const
{
	constexpr double sigma = 0.1f;
	if (period > periodData.size()) { return {}; }
	auto interval = periodData[period];
	auto start = interval.first;
	auto end = start + interval.second;

	if (interval.second < 2) return {};

	auto timeAcc = 0.0;
	auto lastTime = data[start].time;
	auto lastVal = data[start].velocity;
	for (size_t i = start + 1; i < end; i++)
	{
		if (lastVal * (1 - sigma) < data[i].velocity && data[i].velocity < lastVal * (1 + sigma))
		{
			auto timeDiff = (data[i].time - lastTime).count() / 1000.0;
			timeAcc += timeDiff;
			lastTime = data[i].time;
		} else
		{
			return timeAcc;
		}
	}
	return {};
}

void
RelayAnalyzer::calc() const
{
	const auto v_peaks = findPeaksOuter();
	const auto static_osc_peak = findStaticOscillationSampleOuter(v_peaks);
	const auto static_period = findPeriodForSample(static_osc_peak);
	const auto lastLowSample_2 = findLastLowInner(static_period);
	const auto firstHighSample_2 = findSteadyStateInner(static_period);

	const auto omega_u = findUltimateFreqOuter(v_peaks);
	const auto k_2 = computeStaticGainInner(firstHighSample_2, lastLowSample_2);
	const auto [t_2, l_2] = computeDeadTimeAndTimeConstantInner(static_period, k_2);

	const auto l = getDeadTimeOuter(static_period);
	const auto l_1 = l - l_2;

	const auto g_2 = getFreqResponseInner(k_2, l_2, t_2, omega_u);
	const auto g = getFreqResponseWhole(static_period, omega_u);
	const auto g_1 = g / g_2;

	const auto res = g_1 / std::exp(std::complex<double>{0, l_1 * omega_u});

	const auto t_1 = -res.imag() / (res.real() * omega_u);
	const auto k_1 = (res.real() * res.real() + res.imag() * res.imag()) / res.real();
}
