#include "tests.hpp"

#include <modm/debug/logger.hpp>
#include <modm/processing/timer.hpp>
#include <micro-motor/micro-motor.hpp>

#include "../motor.hpp"

bool
test_run_forwards(Motor& Motor0, const MotorInfo& info)
{
	MODM_LOG_INFO << "Testing light load... (5s)" << modm::endl;

	Motor0.setPWM(2000);
	auto testStart = modm::Clock::now();
	Motor0.update();
	auto lastPosition = Motor0.getPosition();
	float avgVelAcc{}, minCurrent{}, maxCurrent{}, avgCurrentAcc{};
	uint32_t avgCurrentNum{}, avgVelNum{};
	bool first = true;
	while (modm::Clock::now() - testStart < 5s)
	{
		modm::delay_ms(1);
		Motor0.update();
		std::array<float, 3> currents = micro_motor::getADCCurrents();

		auto current =
			std::max(std::max(std::abs(currents[0]), std::abs(currents[1])), std::abs(currents[2]));
		if (current < minCurrent || first) { minCurrent = current; }
		if (current > maxCurrent || first) { maxCurrent = current; }
		avgCurrentAcc += current;

		avgCurrentNum++;
		auto pos = Motor0.getPosition();
		auto vel = pos - lastPosition;
		lastPosition = pos;
		avgVelAcc += vel;
		avgVelNum++;
		first = false;
	}
	Motor0.setPWM(0);
	Motor0.update();
	avgVelAcc /= avgVelNum;
	bool success = true;
	MODM_LOG_INFO << "Done!" << modm::endl;
	MODM_LOG_INFO << "Velocity: " << avgVelAcc << modm::endl;
	MODM_LOG_INFO << "Current: [min: " << minCurrent << ",avg: " << avgCurrentAcc
				  << ",max: " << maxCurrent << "]" << modm::endl;

	constexpr float epsilon = 0.1f;

	if (std::abs(avgVelAcc) <= epsilon)
	{
		MODM_LOG_INFO << "Motor did not register movement!" << modm::endl;
		MODM_LOG_INFO << "Check your commutation offset!" << modm::endl;
		success = false;
	}
	if ((avgVelAcc < 0.0f && !info.reversed) || (avgVelAcc > 0.0f && info.reversed))
	{
		MODM_LOG_INFO << "Motor turned the wrong way!" << modm::endl;
		MODM_LOG_INFO << "Check your direction!" << modm::endl;
		success = false;
	}
	if (!success) { MODM_LOG_INFO << "Failed light load test!" << modm::endl; }
	return success;
}
