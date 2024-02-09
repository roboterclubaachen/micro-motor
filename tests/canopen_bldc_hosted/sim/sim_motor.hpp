#pragma once
#include <limits>

#include "motor_bridge.hpp"

namespace sim
{

struct MotorData
{
	double stator_phase_r{11.9};         // Resistance
	double stator_phase_l{0.00207};      // Inductance
	double mutual_inductance{-0.00069};  // Mutual Inductance between Phases
	double inertia{0.000007};            // Moment of inertia
	double k_v = 1000. / 32.3;           // Motor constant
	double damping = inertia / 0.0006;   // Damping constant

	size_t num_poles{4};
};

class MotorSimulation
{
private:
	static inline constexpr MotorData data_{};
	static inline MotorState state_{};

public:
	static double
	angleMod(double angle);

	static Eigen::Vector3d
	emfFunction(double rotor_p);

	static void
	updateVoltages();

	static void
	updateHallPort();

	static void
	update(double timestep);

	static MotorState&
	state();

	static MotorBridge&
	bridge();

	static double
	maxCurrent();
};

}  // namespace sim