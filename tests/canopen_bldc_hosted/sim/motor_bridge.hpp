#pragma once
#include <array>
#include <cstdint>

#include "motor_state.hpp"
#include "hall_port.hpp"

#include <librobots2/motor/motor_bridge.hpp>

namespace sim
{
using librobots2::motor::BridgeConfig;
using librobots2::motor::Phase;
using librobots2::motor::PhaseConfig;

class MotorBridge
{
private:
	static uint8_t
	toIndex(Phase p);

public:
	using HallPort = SimHallPort;

	static constexpr uint16_t MaxPwm = 2047;

	static void
	update(double timestep);

	static void
	initialize();

	static void
	configure(const BridgeConfig& config);

	static void
	configure(PhaseConfig config);

	static void
	configure(Phase phase, PhaseConfig config);

	static void
	setCompareValue(uint16_t compareValue);

	static void
	setCompareValue(Phase phase, uint16_t compareValue);

	static void
	applyCompareValues();

	static std::array<Gate, 3>
	getConfig();

private:
	static inline std::array<PhaseConfig, 3> phaseConfig{PhaseConfig::HiZ, PhaseConfig::HiZ,
														 PhaseConfig::HiZ};
	static inline std::array<Gate, 3> gateConfig{Gate::HiZ, Gate::HiZ, Gate::HiZ};
	static inline std::array<uint16_t, 3> pwms{};
	static inline std::array<double, 3> onTime{};
};

}  // namespace sim