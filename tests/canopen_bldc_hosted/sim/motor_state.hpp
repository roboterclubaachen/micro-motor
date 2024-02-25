#pragma once
#include <array>
#include <Eigen/Dense>

namespace sim
{

struct MotorData
{
	double l{0.0215};           // Phase inductance
	double m{0.002};            // Mutual inductance
	double r_s{11.05};          // Stator resistance
	double k_e{0.0143};         // BackEMF constant
	uint8_t p{6};               // Number of pole pairs
	double vdc{2 * 10 * M_PI};  // Supply voltage

	double j{0.0001};  // Inertia
	double f{0.001};   // Friction
};

struct MotorState
{
	Eigen::Vector3d i{0, 0, 0}, v{0, 0, 0}, e{0, 0, 0};  // Current Voltage and BackEMF
	double omega_m{};                                    // Mechanical angular velocity
	double t_e{};                                        // Electromagnetic Torque
	double t_f{};                                        // Friction Torque
	double t_l{0.0};                                     // Load Torque
	double theta_m{};                                    // Mechanical angle
	double theta_e{};
};

}  // namespace sim