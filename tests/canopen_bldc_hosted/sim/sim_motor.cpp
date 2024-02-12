#include "sim_motor.hpp"
#include <limits>
#include <modm/debug/logger.hpp>
#include <cmath>

namespace sim
{

double
MotorSimulation::angleMod(double angle)
{
	angle = std::fmod(angle, M_PI * 2);
	if (angle < 0.0)
	{
		angle += M_PI * 2;
		angle = std::fmod(angle, M_PI * 2);
	}
	return angle;
}

Eigen::Vector3d
MotorSimulation::emfFunction(double rotor_p)
{
	// Switching Pattern-Independent Simulation Model for Brushless DC Motors (Kang et al. 2011)
	auto out = Eigen::Vector3d();
	for (size_t i = 0; i < 3; i++)
	{
		rotor_p = angleMod(rotor_p);
		if (0 <= rotor_p && rotor_p < M_PI / 6)
		{
			out(i) = rotor_p * 6 / M_PI;
		} else if (M_PI / 6 <= rotor_p && rotor_p < 5 * M_PI / 6)
		{
			out(i) = 1;
		} else if (5 * M_PI / 6 <= rotor_p && rotor_p < 7 * M_PI / 6)
		{
			out(i) = (M_PI - rotor_p) * 6 / M_PI;
		} else if (7 * M_PI / 6 <= rotor_p && rotor_p < 11 * M_PI / 6)
		{
			out(i) = -1;
		} else if (11 * M_PI / 6 <= rotor_p && rotor_p < 2 * M_PI)
		{
			out(i) = (rotor_p - 2 * M_PI) * 6 / M_PI;
		}
		rotor_p += M_PI * 2.0 / 3.0;
	}
	return -out;
}

Eigen::Vector3d
MotorSimulation::computeVoltages(double v, const std::array<Gate, 3>& config,
								 const Eigen::Vector3d& bemf)
{
	size_t acc{0};
	double center{0.0};
	Eigen::Vector3d voltages{0, 0, 0};

	// At this point we technically compute potentials
	for (size_t i = 0; i < config.size(); i++)
	{
		voltages(i) =
			v / 2 *
			(int8_t)config[i];  // Set outside points to +-half VDC depending if Gate is high or low
		if (config[i] != Gate::HiZ)
		{
			acc++;
			center += voltages(i) - bemf(i);  // Add bemf subtracted voltage to midpoint
		}
	}
	center /= acc;  // Compute center voltage

	// Make voltages relative to center
	for (size_t i = 0; i < config.size(); i++)
	{
		if (config[i] == Gate::HiZ)
		{
			voltages(i) = bemf(i);
		} else
		{
			voltages(i) -= center;
		}
	}

	return voltages;
}

MotorState
MotorSimulation::nextState(const std::array<Gate, 3> config, double timestep)
{
	// Trapezoidal function
	const auto emf_factor = emfFunction(state_.omega_m * data_.p / 2);

	// Actual back emf voltages
	const auto e = emf_factor * data_.k_e * state_.omega_m;

	// Phase voltages
	const auto v = computeVoltages(data_.vdc, config, e);

	// Phase Currents
	const auto d_i = (v - data_.r_s * state_.i - e) / (data_.l - data_.m);
	const auto i = state_.i + d_i * timestep;

	// Torque
	const auto t_e = emf_factor.dot(state_.i) * data_.k_e;

	// Mechanics
	const auto d_omega_m = (t_e - state_.t_l - data_.f * state_.omega_m) / data_.j;
	const auto omega_m = state_.omega_m + d_omega_m * timestep;
	const auto theta_m = angleMod(state_.theta_m + state_.omega_m * timestep);

	// Create new state object
	MotorState out{};
	out.e = e;
	out.i = i;
	out.v = v;
	out.omega_m = omega_m;
	out.theta_m = theta_m;
	out.t_e = t_e;
	out.t_l = state_.t_l;
	return out;
}

void
MotorSimulation::update(double timestep)
{
	// Update our bridge config
	MotorBridge::update(timestep);
	state_ = nextState(MotorBridge::getConfig(), timestep);
}

double
MotorSimulation::maxCurrent()
{
	return state_.i.cwiseAbs().maxCoeff();
}

void
MotorSimulation::updateHallPort()
{
	const auto index = ((unsigned int)std::round(angleMod(state_.theta_m) * 6 / (2 * M_PI))) % 6;

	Pin<0>::set(index == 5 || index == 0 || index == 1);
	Pin<1>::set(index == 1 || index == 2 || index == 3);
	Pin<2>::set(index == 3 || index == 4 || index == 5);
}

MotorState&
MotorSimulation::state()
{
	return state_;
}

}  // namespace sim