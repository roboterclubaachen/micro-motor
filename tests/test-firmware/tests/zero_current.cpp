#include "tests.hpp"
#include <modm/debug/logger.hpp>
#include <modm/processing/timer.hpp>
#include <micro-motor/micro-motor.hpp>

bool
test_zero_current(Motor &, const MotorInfo &)
{

	MODM_LOG_INFO << "Measuring current at no load... (2s)" << modm::endl;

	bool first = true;
	std::array<float, 3> minCurrent{}, maxCurrent{}, avgCurrentAcc{};
	uint32_t avgCurrentNum{};

	auto testStart = modm::Clock::now();
	while (modm::Clock::now() - testStart < 2s)
	{
		std::array<float, 3> currents = micro_motor::getADCCurrents();
		for (size_t i = 0; i < 3; i++)
		{
			if (currents[i] < minCurrent[i] || first) { minCurrent[i] = currents[i]; }
			if (currents[i] > maxCurrent[i] || first) { maxCurrent[i] = currents[i]; }
			avgCurrentAcc[i] += currents[i];
		}
		avgCurrentNum++;
		first = false;
		modm::delay_us(100);
	}
	constexpr float epsilon = 0.1f;

	avgCurrentAcc[0] /= avgCurrentNum;
	avgCurrentAcc[1] /= avgCurrentNum;
	avgCurrentAcc[2] /= avgCurrentNum;

	MODM_LOG_INFO << "Zero Current test done!" << modm::endl;
	MODM_LOG_INFO << "Currents: [min,avg,max]" << modm::endl;

	MODM_LOG_INFO << "Phase U: [" << minCurrent[0] << "," << (avgCurrentAcc[0]) << ","
				  << maxCurrent[0] << "]" << modm::endl;

	MODM_LOG_INFO << "Phase V: [" << minCurrent[1] << "," << (avgCurrentAcc[1]) << ","
				  << maxCurrent[1] << "]" << modm::endl;

	MODM_LOG_INFO << "Phase W: [" << minCurrent[2] << "," << (avgCurrentAcc[2]) << ","
				  << maxCurrent[2] << "]" << modm::endl;

	bool success = true;
	for (size_t i = 0; i < 3; i++)
	{
		char phase = (i == 0 ? 'U' : (i == 2 ? 'V' : 'W'));
		if (minCurrent[i] < -epsilon)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " minimum current is bigger than allowed!"
						  << modm::endl;
			MODM_LOG_INFO << minCurrent[i] << " < " << -epsilon << modm::endl;
		}
		if (maxCurrent[i] > epsilon)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " maximum current is bigger than allowed!"
						  << modm::endl;
			MODM_LOG_INFO << maxCurrent[i] << " > " << epsilon << modm::endl;
		}
		if (avgCurrentAcc[i] < -epsilon / 2.0f || avgCurrentAcc[i] > epsilon / 2.0f)
		{
			success = false;
			MODM_LOG_INFO << "Phase " << phase << " average current is outside of allowed window!"
						  << modm::endl;
			MODM_LOG_INFO << avgCurrentAcc[i] << " < " << -epsilon / 2.0f << " || "
						  << avgCurrentAcc[i] << " > " << epsilon / 2.0f << modm::endl;
		}
	}
	if (!success) { MODM_LOG_INFO << "Failed zero current test!" << modm::endl; }
	return success;
}