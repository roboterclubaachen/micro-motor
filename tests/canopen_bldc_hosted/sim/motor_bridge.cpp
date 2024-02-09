#include "motor_bridge.hpp"

#include <modm/debug/logger.hpp>

namespace sim
{

uint8_t
MotorBridge::toIndex(Phase p)
{
	switch (p)
	{
		case Phase::PhaseU:
			return 0;
		case Phase::PhaseV:
			return 1;
		case Phase::PhaseW:
			return 2;
		default:
			return 0;
	}
}

void
MotorBridge::initialize()
{
	phaseConfig = {PhaseConfig::HiZ, PhaseConfig::HiZ, PhaseConfig::HiZ};
	gateConfig = {Gate::HiZ, Gate::HiZ, Gate::HiZ};
	pwms = {};
	onTime = {};
}

void
MotorBridge::configure(const BridgeConfig& config)
{
	phaseConfig[toIndex(Phase::PhaseU)] = config.get(Phase::PhaseU);
	phaseConfig[toIndex(Phase::PhaseV)] = config.get(Phase::PhaseV);
	phaseConfig[toIndex(Phase::PhaseW)] = config.get(Phase::PhaseW);
}

void
MotorBridge::configure(PhaseConfig config)
{
	for (auto& p : phaseConfig) { p = config; }
}

void
MotorBridge::configure(Phase phase, PhaseConfig config)
{
	auto index = toIndex(phase);
	phaseConfig[index] = config;
}

void
MotorBridge::setCompareValue(uint16_t compareValue)
{
	for (auto& pwm : pwms) { pwm = compareValue; }
}

void
MotorBridge::setCompareValue(Phase phase, uint16_t compareValue)
{
	auto index = toIndex(phase);
	pwms[index] = compareValue;
}

void
MotorBridge::applyCompareValues()
{}

std::array<Gate, 3>
MotorBridge::getConfig()
{
	return gateConfig;
}

void
MotorBridge::update(double timestep)
{
	for (size_t i = 0; i < 3; i++)
	{
		if (phaseConfig[i] != PhaseConfig::Pwm)
		{
			auto g = Gate::HiZ;
			switch (phaseConfig[i])
			{
				case PhaseConfig::High:
					g = Gate::High;
					break;
				case PhaseConfig::Low:
					g = Gate::Low;
					break;
				default:
					break;
			}
			gateConfig[i] = g;
		} else
		{
			constexpr double pwmPeriod = 0.00001;  // 10us
			// TODO use the nanosecond clock to determine timestep if its too coarse
			// Maybe implement constexpr double timeScale = 0.0001 on all timers or something?

			onTime[i] += timestep;

			const double ratio = (double)pwms[i] / MaxPwm;

			double maxOnTime = 0.0;
			if (gateConfig[i] == Gate::High) { maxOnTime = pwmPeriod * ratio; }
			if (gateConfig[i] == Gate::Low) { maxOnTime = pwmPeriod * (1 - ratio); }

			if (onTime[i] >= maxOnTime)
			{
				onTime[i] = 0.0;
				gateConfig[i] = (gateConfig[i] == Gate::High ? Gate::Low : Gate::High);
			}
		}
	}
	MODM_LOG_DEBUG << "Switches " << (uint8_t)phaseConfig[0] << " " << (uint8_t)phaseConfig[1]
				   << " " << (uint8_t)phaseConfig[2] << modm::endl;

	MODM_LOG_DEBUG << "pwm " << pwms[0] << " " << pwms[1] << " " << pwms[2] << modm::endl;
}

}  // namespace sim