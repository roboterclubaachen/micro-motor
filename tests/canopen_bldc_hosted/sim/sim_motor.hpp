#pragma once
#include <limits>

#include "motor_bridge.hpp"

namespace sim
{

class MotorSimulation
{
private:
	static inline const MotorData data_{};
	static inline MotorState state_{};

	static Eigen::Vector3d
	computeVoltages(double v, const std::array<float, 3>& pwms,
					const std::array<PhaseConfig, 3>& config, const Eigen::Vector3d& bemf);

	static MotorState
	nextState(const std::array<float, 3>& pwms, const std::array<PhaseConfig, 3>& config,
			  double timestep);

	static double
	angleMod(double angle);

	static Eigen::Vector3d
	emfFunction(double rotor_p);

	static void
	updateHallPort();

public:
	static void
	update(double timestep);

	static MotorState&
	state();

	static double
	maxCurrent();
};

}  // namespace sim