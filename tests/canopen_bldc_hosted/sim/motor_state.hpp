#pragma once
#include <array>
#include <Eigen/Dense>

namespace sim
{

enum class Gate : uint8_t
{
	HiZ = 0,
	Low = 1,
	High = 2
};

struct MotorState
{
	double supply_v{100};
	double loadTorque{0.0};
	double friction{0.0};

	std::array<Gate, 3> switches{Gate::HiZ, Gate::HiZ, Gate::HiZ};

	double rotor_theta{0.0}, rotor_omega{0.0}, rotor_omega_dot{0.0};
	Eigen::Vector3d phase_v{0, 0, 0}, phase_i{0, 0, 0};
	double star_v{0.0};
};

}  // namespace sim