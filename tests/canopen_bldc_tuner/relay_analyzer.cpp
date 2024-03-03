#include "relay_analyzer.hpp"
#include <Eigen/Dense>

#include <modm/debug/logger.hpp>
#include <cmath>
#include "csv_writer.hpp"

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
			currentPeriod++;
		}
	}
	out.push_back({index, size});
	return out;
}

void
RelayAnalyzer::setData(const std::vector<RelayUpdate>& data)
{
	MODM_LOG_INFO << "Sample count: " << data.size() << modm::endl;
	for (size_t i = 0; i < data.size(); i++)
	{
		currents.push_back(data[i].current);
		demands.push_back(data[i].demand);
		velocities.push_back(data[i].velocity);
		times.push_back(data[i].time);
	}
	periodData = getPeriodInfo(data);
	MODM_LOG_INFO << "Period count: " << periodData.size() << modm::endl;
}
size_t
RelayAnalyzer::findSteadyState(const std::vector<double>& values, Period period,
							   size_t lastLowSample)
{
	constexpr double sigma = 0.5f;
	constexpr auto epsilon = 5;
	if (lastLowSample >= values.size()) { return values.size(); }

	auto start = period.first;
	auto end = start + period.second;

	if (period.second < 2) { return values.size(); }

	size_t index = lastLowSample;
	double value = values[index];
	for (size_t i = lastLowSample + 1; i < end; i++)
	{
		if (values[i] > value)
		{
			value = values[i];
			index = i;
		}
		if (values[i] * (1 + sigma) < value && values[i] + epsilon < value) { return index; }
	}
	if (index != lastLowSample) return index;
	return values.size();
}

size_t
RelayAnalyzer::findDemandStart(const std::vector<double>& demand, Period period)
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

size_t
RelayAnalyzer::findLastLow(const std::vector<size_t>& valleys,
						   const std::vector<Period>& periodData, Period period)
{
	for (auto val : valleys)
	{
		auto val_period = findPeriodForSample(periodData, val);
		if (val_period == period) return val;
	}
	return periodData.size();
}

double
RelayAnalyzer::computeStaticGain(const std::vector<double>& values,
								 const std::vector<double>& demand, size_t highSampleIndex,
								 size_t lowSampleIndex)
{
	if (highSampleIndex >= values.size()) { return 0.0f; }
	if (lowSampleIndex >= values.size()) { return 0.0f; }

	auto currentHigh = values[highSampleIndex];
	auto currentLow = values[lowSampleIndex];
	auto demandHigh = demand[highSampleIndex];
	auto demandLow = demand[lowSampleIndex];

	auto demandDiff = demandHigh - demandLow;
	auto currentDiff = currentHigh - currentLow;

	return currentDiff / demandDiff;
}

std::pair<double, double>
RelayAnalyzer::computeDeadTimeAndTimeConstantInner(Period period, double static_gain,
												   size_t lowSampleIndex,
												   size_t highSampleIndex) const
{

	// const auto deadTimeEstimate = getDeadTimeOuter(demands, times, period, lowSampleIndex);
	// MODM_LOG_INFO << "Pre estimated inner dead time: " << deadTimeEstimate << modm::endl;

	using MatrixX2d = Eigen::Matrix<double, Eigen::Dynamic, 2>;
	using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

	double a = 0.0f;
	const auto demandSpike = findDemandStart(demands, period);

	/*for (size_t i = demandSpike; i < lowSampleIndex; i++)
	{
		auto current = currents[i];
		auto timeSinceLast = std::chrono::duration_cast<std::chrono::duration<float>>(
			times[i] - times[i == 0 ? 0 : i - 1]);
		a += timeSinceLast.count() * current;
		MODM_LOG_INFO << "I[" << i << "]: " << current << " dt[" << i
					  << "]: " << timeSinceLast.count() << "s A[" << i << "]:" << a << modm::endl;
	}*/

	const auto modifiedLength = highSampleIndex - demandSpike;
	MatrixX2d psi = MatrixX2d(modifiedLength, 2);
	VectorXd gamma = VectorXd(modifiedLength, 1);

	for (size_t i = demandSpike; i < highSampleIndex; i++)
	{
		const auto mi = i - demandSpike;  // Matrix index

		auto timeSinceLast = std::chrono::duration_cast<std::chrono::duration<float>>(
			times[i] - times[(i == 0 ? 0 : i - 1)]);

		auto timeSinceStart =
			std::chrono::duration_cast<std::chrono::duration<float>>(times[i] - times[demandSpike]);

		auto current = currents[i];
		a += timeSinceLast.count() * current;

		psi(mi, 0) = current;
		psi(mi, 1) = static_gain;
		gamma(mi) = static_gain * timeSinceStart.count() - a;

		// MODM_LOG_INFO << "Psi: " << psi(mi, 0) << ":" << psi(mi, 1) << " ";
		// MODM_LOG_INFO << "Gamma: " << gamma(mi) << " ";
		// MODM_LOG_INFO << "t[" << i << "]: " << timeSinceStart.count() << "s dt[" << i
		//			  << "]: " << timeSinceLast.count() << "s A[" << i << "]:" << a << modm::endl;
	}

	auto XStar = (psi.transpose() * psi).inverse() * psi.transpose() * gamma;
	return {XStar(0, 0), XStar(1, 0)};
}

double
RelayAnalyzer::computeWindowAverage(const std::vector<double>& values, size_t i, size_t windowSize)
{
	auto hWS = windowSize / 2;
	if (hWS * 2 < windowSize) hWS++;
	double windowAvg = 0.0f;
	for (size_t j = i - hWS; j < i - hWS + windowSize; j++) { windowAvg += values[j]; }
	return windowAvg / windowSize;
}

std::vector<size_t>
RelayAnalyzer::findPeaks(const std::vector<double>& values)
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
	constexpr size_t localMaxWindow = 100;

	if (averages.size() < stride * 2 + 1) return {};
	bool plateaud = false;
	size_t plateauStartIndex = 0;
	double plateauStartValue = 0.0;
	for (size_t i = stride; i < averages.size() - stride; i++)
	{
		double pastMax{0.0}, futureMax{0.0};
		for (size_t j = i - 1; j > i - localMaxWindow; j--)
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

std::vector<size_t>
RelayAnalyzer::findValleys(const std::vector<double>& values)
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
	constexpr size_t localMinWindow = 500;

	if (averages.size() < stride * 2 + 1) return {};
	bool plateaud = false;
	size_t plateauStartIndex = 0;
	double plateauStartValue = 0.0;
	for (size_t i = stride; i < averages.size() - stride; i++)
	{
		double pastMin{averages[i - 1]}, futureMin{averages[i + 1]};
		for (size_t j = i - 1; j > i - localMinWindow; j--)
		{
			if (averages[j] < pastMin) pastMin = averages[j];
			if (j == 0) break;
		}
		for (size_t j = i + 1; j < i + localMinWindow && j < averages.size(); j++)
		{
			if (averages[j] < futureMin) futureMin = averages[j];
		}

		auto curr = averages[i];
		if (pastMin >= curr)
		{
			plateauStartIndex = i;
			plateauStartValue = curr;
			plateaud = true;
		}

		if (plateauStartValue < futureMin)
		{
			if (plateaud) { out.push_back(plateauStartIndex + halfWindowSize); }
			plateaud = false;
		}
	}
	return out;
}

double
RelayAnalyzer::findUltimateFreqOuter(const std::vector<size_t>& peaks,
									 const std::vector<modm::Clock::time_point>& time)
{
	if (peaks.size() == 0) return 0.0;
	size_t count = 0;
	modm::Clock::duration acc(0);
	auto lastTime = time[peaks[0]];
	for (size_t i = 1; i < peaks.size(); i++)
	{
		auto currTime = time[peaks[i]];
		auto diff = currTime - lastTime;
		lastTime = currTime;
		acc += diff;
		count++;
	}
	acc /= count;
	return 1000.0f * M_PI * 2 / (double)acc.count();
}
uint64_t
RelayAnalyzer::findStaticOscillationSample(const std::vector<size_t>& peaks,
										   const std::vector<double>& values,
										   const std::vector<modm::Clock::time_point>& time)
{
	constexpr double sigmaT = 0.1f, sigmaY = 0.1f;

	if (peaks.size() <= 2) return values.size();
	const size_t minCountInWindow = std::min<size_t>(5, peaks.size() - 1);

	auto tDiff =
		std::chrono::duration_cast<std::chrono::duration<float>>(time[peaks[1]] - time[peaks[0]])
			.count();
	auto yDiff = (values[peaks[1]] - values[peaks[0]]);

	size_t inWindow = 0;

	for (size_t i = 2; i < peaks.size(); i++)
	{
		const auto tDiffMax = std::max(tDiff * (1.0 + sigmaT), tDiff * (1.0 - sigmaT));
		const auto tDiffMin = std::min(tDiff * (1.0 + sigmaT), tDiff * (1.0 - sigmaT));

		const auto yDiffMax = std::max(yDiff * (1.0 + sigmaY), yDiff * (1.0 - sigmaY));
		const auto yDiffMin = std::min(yDiff * (1.0 + sigmaY), yDiff * (1.0 - sigmaY));

		auto next_tDiff = std::chrono::duration_cast<std::chrono::duration<float>>(
							  time[peaks[i]] - time[peaks[i - 1]])
							  .count();
		auto next_yDiff = (values[peaks[i]] - values[peaks[i - 1]]);

		if (tDiffMin <= tDiff && tDiff <= tDiffMax && yDiffMin <= yDiff && yDiff <= yDiffMax)
		{
			inWindow++;
			if (inWindow >= minCountInWindow) { return peaks[i - 1]; }
		} else
		{
			inWindow = 0;
		}

		tDiff = next_tDiff;
		yDiff = next_yDiff;
	}
	return values.size();
}

RelayAnalyzer::Period
RelayAnalyzer::findPeriodForSample(const std::vector<Period>& periodData, size_t sample)
{
	for (size_t i = 0; i < periodData.size(); i++)
	{
		if (periodData[i].first <= sample && sample <= periodData[i].first + periodData[i].second)
		{
			return periodData[i];
		}
	}
	return {};
}
std::complex<double>
RelayAnalyzer::getFreqResponseWhole(const std::vector<double>& demands,
									const std::vector<double>& values,
									const std::vector<modm::Clock::time_point>& time, Period period,
									double omega_u)

{
	const auto startIndex = findDemandStart(demands, period);
	const modm::chrono::milli_clock::time_point startTime = time[startIndex];
	modm::chrono::milli_clock::time_point lastTime = startTime;

	double timeAcc = 0.0;
	std::complex<double> inAcc = {}, outAcc = {};
	const auto t_u = 2.0 * M_PI / omega_u;
	size_t i = startIndex + 1;
	while (timeAcc < t_u && i < values.size())
	{
		const auto t =
			std::chrono::duration_cast<std::chrono::duration<float>>(time[i] - time[startIndex])
				.count();
		const auto timeDiff =
			std::chrono::duration_cast<std::chrono::duration<float>>(time[i] - time[i - 1]).count();
		timeAcc += timeDiff;
		std::complex<double> exponent{};
		exponent.real(0.0);
		exponent.imag(-omega_u * t);
		inAcc += demands[i] * timeDiff * std::exp(exponent);
		outAcc += values[i] * timeDiff * std::exp(exponent);
		lastTime = time[i];
		i++;
	}
	if (i == values.size()) { return {}; }

	return (outAcc) / (inAcc);
}

std::complex<double>
RelayAnalyzer::getFreqResponseInner(double k_2, double l_2, double t_2, double omega_u)
{
	return (k_2 / (std::complex<double>{1, t_2 * omega_u})) *
		   std::exp(std::complex<double>{0, -l_2 * omega_u});
}
double
RelayAnalyzer::getDeadTimeOuter(const std::vector<double>& demand,
								const std::vector<modm::Clock::time_point>& time, Period period,
								size_t lastLowSample)
{
	const auto demandSpike = findDemandStart(demand, period);
	// MODM_LOG_INFO << "Last Low: " << time[lastLowSample].time_since_epoch().count() <<
	// modm::endl; MODM_LOG_INFO << "Demand Spike: " << time[demandSpike].time_since_epoch().count()
	// << modm::endl;
	return std::chrono::duration_cast<std::chrono::duration<float>>(time[lastLowSample] -
																	time[demandSpike])
		.count();
}

template<size_t n>
std::array<double, n>
computePade(const std::array<double, n>& d, const std::array<double, n>& e)
{
	std::array<double, n> c = {};
	c[0] = d[0] / e[0];
	for (size_t k = 1; k < n; k++)
	{
		double temp = 0.0;
		for (size_t j = 0; j < k; j++) { temp += e[j] * c[k - j]; }
		c[k] = (d[k] - temp) / e[0];
	}
	return c;
}

bool
RelayAnalyzer::calc() const
{

	// Implementing "Auto-tuning of cascade control systems", Song et al. 2002
	// https://www.sciencedirect.com/science/article/pii/S0019057807601141
	analysis = {};
	MODM_LOG_INFO << "Finding peaks and valleys in velocity..." << modm::endl;
	const auto v_peaks = findPeaks(velocities);
	const auto v_valleys = findValleys(velocities);
	MODM_LOG_INFO << "Found " << v_peaks.size() << " Peaks and " << v_valleys.size() << " Valleys."
				  << modm::endl;
	if (v_peaks.size() == 0 || v_valleys.size() == 0) return false;
	analysis.vel_peaks = v_peaks;
	analysis.vel_valleys = v_valleys;

	MODM_LOG_INFO << "Finding valleys in current..." << modm::endl;
	const auto c_valleys = findValleys(currents);
	MODM_LOG_INFO << "Found " << c_valleys.size() << " Valleys." << modm::endl;
	if (c_valleys.size() == 0) return false;
	analysis.cur_valleys = c_valleys;
	for (size_t i = 0; i < periodData.size(); i++)
	{
		Period p = findPeriodForSample(periodData, analysis.cur_valleys[i]);
		auto sample = findSteadyState(currents, p, analysis.cur_valleys[i]);
		if (sample < currents.size()) analysis.cur_peaks.push_back(sample);
	}

	MODM_LOG_INFO << "Finding Static oscillation Sample..." << modm::endl;
	const auto static_osc_peak = findStaticOscillationSample(v_peaks, velocities, times);
	const auto static_period = findPeriodForSample(periodData, static_osc_peak);
	MODM_LOG_INFO << "Found Static oscillation in period " << static_period.first << ":"
				  << static_period.second << "." << modm::endl;
	if (static_period.first == 0 && static_period.second == 0) return false;

	MODM_LOG_INFO << "Analyzing oscillation period..." << modm::endl;
	const auto lastLowSample_2 = findLastLow(c_valleys, periodData, static_period);
	const auto lastLowDemandSample_2 = findDemandStart(demands, static_period);
	const auto firstHighSample_2 = findSteadyState(currents, static_period, lastLowSample_2);

	MODM_LOG_INFO << "Last low demand sample: " << lastLowDemandSample_2 << modm::endl;
	MODM_LOG_INFO << "Last low response sample: " << lastLowSample_2 << modm::endl;
	MODM_LOG_INFO << "First steady state sample: " << firstHighSample_2 << modm::endl;
	if (lastLowSample_2 >= currents.size() || firstHighSample_2 >= currents.size() ||
		lastLowDemandSample_2 >= currents.size())
		return false;

	MODM_LOG_INFO << "Finding ultimate Frequency..." << modm::endl;
	const auto omega_u = findUltimateFreqOuter(v_peaks, times);
	MODM_LOG_INFO << "Ultimate Frequency: " << omega_u << modm::endl;
	if (omega_u == 0.0) return false;

	MODM_LOG_INFO << "Computing inner static gain..." << modm::endl;
	const auto k_2 = computeStaticGain(currents, demands, firstHighSample_2, lastLowDemandSample_2);
	MODM_LOG_INFO << "Inner static gain: " << k_2 << modm::endl;

	MODM_LOG_INFO << "Computing inner dead time and time constant..." << modm::endl;
	const auto [t_2, l_2] =
		computeDeadTimeAndTimeConstantInner(static_period, k_2, lastLowSample_2, firstHighSample_2);

	MODM_LOG_INFO << "Finding dead times..." << modm::endl;
	const auto lastLowSample_1 = findLastLow(v_valleys, periodData, periodData[0]);
	const auto l = getDeadTimeOuter(demands, times, periodData[0], lastLowSample_1);
	const auto l_1 = l - l_2;

	MODM_LOG_INFO << "Inner dead time: " << l_2 << modm::endl;
	MODM_LOG_INFO << "Outer dead time: " << l_1 << modm::endl;
	MODM_LOG_INFO << "Dead time whole: " << l << modm::endl;

	MODM_LOG_INFO << "Computing frequency response..." << modm::endl;
	const auto g_2 = getFreqResponseInner(k_2, l_2, t_2, omega_u);
	const auto g_whole = getFreqResponseWhole(demands, velocities, times, static_period, omega_u);
	const auto g_1 = g_whole / g_2;
	const auto g_1_prime = g_1 / std::exp(std::complex<double>{0, -l_1 * omega_u});

	MODM_LOG_INFO << "Frequency response whole: " << g_whole.real() << " + " << g_whole.imag()
				  << "i" << modm::endl;
	MODM_LOG_INFO << "Inner frequency response: " << g_2.real() << " + " << g_2.imag() << "i"
				  << modm::endl;
	MODM_LOG_INFO << "Outer frequency response: " << g_1.real() << " + " << g_1.imag() << "i"
				  << modm::endl;

	MODM_LOG_INFO << "Inner frequency response without dead time: " << g_1_prime.real() << " + "
				  << g_1_prime.imag() << "i" << modm::endl;

	const auto t_1 = -g_1_prime.imag() / (g_1_prime.real() * omega_u);
	const auto k_1 = (g_1_prime.real() * g_1_prime.real() + g_1_prime.imag() * g_1_prime.imag()) /
					 g_1_prime.real();

	MODM_LOG_INFO << "Process parameters:" << modm::endl;
	MODM_LOG_INFO << "K_1 " << k_1 << " T_1 " << t_1 << " L_1 " << l_1 << modm::endl;
	MODM_LOG_INFO << "K_2 " << k_2 << " T_2 " << t_2 << " L_2 " << l_2 << modm::endl;

	// Maybe just use Cohen Coon here?
	// https://blog.opticontrols.com/wp-content/uploads/2011/03/CohenCoonRules1.png
	// TODO Debug after this point!

	const auto k_p2 = 0.7 * t_2 / (k_2 * l_2);
	const auto k_i2 = 0.304 * t_2 / (k_2 * l_2);

	const auto damping = 0.707;
	const auto omega_n = 0.8 * omega_u;

	std::array<double, 5> d = {omega_n * omega_n, 0.0, 0.0, 0.0, 0.0};
	std::array<double, 5> e = {omega_n * omega_n, 2.0 * damping * omega_n, 1.0, 0.0, 0.0};
	std::array<double, 4> g = {
		k_1 * k_2 * k_i2,
		k_1 * k_2 * (k_p2 - k_i2 * l),
		k_1 * k_2 * (k_i2 * l * l / 2 - k_p2 * l),
		k_1 * k_2 * k_p2 * l * l / 2,
	};
	std::array<double, 4> h = {
		k_2 * k_i2,
		k_2 * t_1 * k_i2 + 1 + k_2 * k_p2 - k_2 * k_i2 * l_2,
		t_1 * (1 + k_2 * k_p2 - k_2 * k_i2 * l_2) + t_2 - k_2 * k_p2 * l_2,
		t_1 * (t_2 - k_2 * k_p2 * l_2),
	};

	// Pade coefficients
	std::array<double, 5> c = computePade(d, e);

	using Matrix5d = Eigen::Matrix<double, 5, 5>;
	using Vector5d = Eigen::Matrix<double, 5, 1>;

	Matrix5d matrix = Matrix5d::Zero();
	matrix(0, 0) = g[0] * c[1];
	matrix(0, 3) = h[0];

	matrix(1, 0) = g[0] * c[2] + g[2] * c[0];
	matrix(1, 1) = g[0] * c[1];
	matrix(1, 3) = h[0] * c[1] + h[1];
	matrix(1, 4) = h[0] * c[0];

	matrix(2, 0) = g[0] * c[3] + g[1] * c[2] + g[2] * c[1];
	matrix(2, 1) = g[0] * c[2] + g[1] * c[1];
	matrix(2, 2) = g[0] * c[1];
	matrix(2, 3) = h[0] * c[2] + h[1] * c[1] + h[2];
	matrix(2, 4) = h[0] * c[1] + h[1];

	matrix(3, 0) = g[0] * c[4] + g[1] * c[3] + g[2] * c[2] + g[3] * c[1];
	matrix(3, 1) = g[0] * c[3] + g[1] * c[2] + g[2] * c[1];
	matrix(3, 2) = g[0] * c[2] + g[1] * c[1];
	matrix(3, 3) = h[0] * c[3] + h[1] * c[2] + h[2] * c[1] + h[3];
	matrix(3, 4) = h[0] * c[2] + h[1] * c[1] + h[2];

	matrix(4, 2) = g[3];
	matrix(4, 4) = h[3];

	Vector5d vector = Vector5d::Zero();
	vector(4) = 1.0;

	auto a = matrix.colPivHouseholderQr().solve(vector);

	const auto k_p1 = (a(1) * a(3) - a(0) * a(4)) / (a(3) * a(3));
	const auto k_i1 = a(0) / a(3);
	const auto k_d1 =
		(a(2) * a(3) * a(3) - a(1) * a(3) * a(4) + a(0) * a(4) * a(4)) / (a(3) * a(3) * a(3));
	const auto k_tn1 = a(4) / a(3);

	MODM_LOG_INFO << "Inner Loop: P " << k_p2 << " I " << k_i2 << " D 0.0" << modm::endl;
	MODM_LOG_INFO << "Outer Loop: P " << k_p1 << " I " << k_i1 << " D " << k_d1 << " Tn " << k_tn1
				  << modm::endl;
	return true;
}

void
RelayAnalyzer::dumpToCSV() const
{
	{
		CSVWriter writer({"Time", "CurrentDemand", "CurrentActual", "CurrentVelocity"});
		if (!writer.create("relay_vel_peaks.csv"))
		{
			MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
			return;
		}
		for (size_t i = 0; i < analysis.vel_peaks.size(); i++)
		{
			auto sample = analysis.vel_peaks[i];
			writer.addRow({std::to_string(times[sample].time_since_epoch().count()),
						   std::to_string(demands[sample]), std::to_string(currents[sample]),
						   std::to_string(velocities[sample])});
		}
		writer.close();
	}
	{
		CSVWriter writer({"Time", "CurrentDemand", "CurrentActual", "CurrentVelocity"});
		if (!writer.create("relay_cur_peaks.csv"))
		{
			MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
			return;
		}
		for (size_t i = 0; i < analysis.cur_peaks.size(); i++)
		{
			auto sample = analysis.cur_peaks[i];
			writer.addRow({std::to_string(times[sample].time_since_epoch().count()),
						   std::to_string(demands[sample]), std::to_string(currents[sample]),
						   std::to_string(velocities[sample])});
		}
		writer.close();
	}
	{
		CSVWriter writer({"Time", "CurrentDemand", "CurrentActual", "CurrentVelocity"});
		if (!writer.create("relay_cur_valleys.csv"))
		{
			MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
			return;
		}
		for (size_t i = 0; i < analysis.cur_valleys.size(); i++)
		{
			auto sample = analysis.cur_valleys[i];
			writer.addRow({std::to_string(times[sample].time_since_epoch().count()),
						   std::to_string(demands[sample]), std::to_string(currents[sample]),
						   std::to_string(velocities[sample])});
		}
		writer.close();
	}
	{
		CSVWriter writer({"Time", "CurrentDemand", "CurrentActual", "CurrentVelocity"});
		if (!writer.create("relay_vel_valleys.csv"))
		{
			MODM_LOG_ERROR << "Could not write csv data." << modm::endl;
			return;
		}
		for (size_t i = 0; i < analysis.vel_valleys.size(); i++)
		{
			auto sample = analysis.vel_valleys[i];
			writer.addRow({std::to_string(times[sample].time_since_epoch().count()),
						   std::to_string(demands[sample]), std::to_string(currents[sample]),
						   std::to_string(velocities[sample])});
		}
		writer.close();
	}
}
