#include "tests.hpp"
#include <modm/debug/logger.hpp>
#include <modm/processing/timer.hpp>
#include <micro-motor/micro-motor.hpp>

bool
test_current_limit(Motor &Motor0, const MotorInfo &)
{
	MODM_LOG_INFO << "Testing Current limit... (2s)" << modm::endl;


	bool first = true;
	std::array<float, 3> maxCurrent{};
	constexpr float currentLimit = 0.01f;

	auto waitStart = modm::Clock::now();

	Motor0.setPWM(8000);
	MODM_LOG_INFO << "Waiting two seconds..." << modm::endl;
	while (modm::Clock::now() - waitStart < 2s){
		Motor0.update();
	};

	MODM_LOG_INFO << "Limiting to: " << currentLimit << "A" << modm::endl;
	micro_motor::setCurrentLimitAmps(currentLimit);

	Motor0.update();
	auto testStart = modm::Clock::now();
	bool abort = false;
	while (modm::Clock::now() - testStart < 2s && !abort)
	{
		modm::delay_ms(1);
		Motor0.update();
		std::array<float, 3> currents = micro_motor::getADCCurrents();
		for (size_t i = 0; i < 3; i++)
		{
			if (currents[i] > maxCurrent[i] || first) { maxCurrent[i] = currents[i]; }
			if (maxCurrent[i] > currentLimit)
			{
				MODM_LOG_ERROR << "Detected over current!" << modm::endl;
				abort = true;
			}
		}
		first = false;
	}

	Motor0.setPWM(0);

	MODM_LOG_INFO << "Current test done!" << modm::endl;
	MODM_LOG_INFO << "Maximum detected Currents: " << modm::endl;

	MODM_LOG_INFO << "Phase U: [" << maxCurrent[0] << "]" << modm::endl;
	MODM_LOG_INFO << "Phase V: [" << maxCurrent[1] << "]" << modm::endl;
	MODM_LOG_INFO << "Phase W: [" << maxCurrent[2] << "]" << modm::endl;

	bool success = true;
	for (size_t i = 0; i < 3; i++)
	{
		char phase = (i == 0 ? 'U' : (i == 2 ? 'V' : 'W'));
		if (maxCurrent[i] > currentLimit)
		{
			MODM_LOG_ERROR << "Detected over current Phase " << phase << "!" << modm::endl;
			MODM_LOG_ERROR << "Detected:" << maxCurrent[i] << "A Allowed: " << currentLimit << "A"
						   << modm::endl;
			success = false;
			break;
		}
	}
	if (!success) { MODM_LOG_INFO << "Failed current limit test!" << modm::endl; }
	return success;
}