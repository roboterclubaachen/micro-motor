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
	return out;
}

void
MotorSimulation::updateVoltages()
{
	// Switching Pattern-Independent Simulation Model for Brushless DC Motors (Kang et al. 2011)
	const auto bemfConstant = 30.0 / (data_.k_v * M_PI);
	auto backEMF = bemfConstant * emfFunction(state_.rotor_theta * (data_.num_poles / 2));

	Eigen::Vector3d voltages{0, 0, 0};
	double star_v = 0.0;
	size_t div = 0;

	if (state_.switches[0] != Gate::HiZ)
	{
		voltages(0) = state_.supply_v / 2 * (state_.switches[0] == Gate::High ? 1 : -1);
		star_v += voltages(0) - backEMF(0);
		div += 1;
	}
	if (state_.switches[1] != Gate::HiZ)
	{
		voltages(1) = state_.supply_v / 2 * (state_.switches[1] == Gate::High ? 1 : -1);
		star_v += voltages(1) - backEMF(1);
		div += 1;
	}
	if (state_.switches[2] != Gate::HiZ)
	{
		voltages(2) = state_.supply_v / 2 * (state_.switches[2] == Gate::High ? 1 : -1);
		star_v += voltages(2) - backEMF(2);
		div += 1;
	}
	if (div != 0) star_v /= div;

	if (state_.switches[0] == Gate::HiZ) { voltages(0) = backEMF(0) + star_v; }
	if (state_.switches[1] == Gate::HiZ) { voltages(1) = backEMF(1) + star_v; }
	if (state_.switches[2] == Gate::HiZ) { voltages(2) = backEMF(2) + star_v; }

	state_.phase_v = voltages;
	state_.star_v = star_v;
}

void
MotorSimulation::update(double timestep)
{
	// Update our bridge config
	MotorBridge::update(timestep);
	state_.switches = MotorBridge::getConfig();

	// Switching Pattern-Independent Simulation Model for Brushless DC Motors
	// (Kang et al. 2011)
	const auto bemfConstant = 30.0 / (data_.k_v * M_PI);
	const auto bemfFactors = emfFunction(state_.rotor_theta * (data_.num_poles / 2));
	const auto backEMF = bemfConstant * bemfFactors;
	MODM_LOG_DEBUG << "BackEMF " << backEMF(0) << " " << backEMF(1) << " " << backEMF(2)
				   << modm::endl;

	double e_torque =
		state_.rotor_omega == 0.0 ? 0.0 : (backEMF.dot(state_.phase_i)) / state_.rotor_omega;

	double m_torque = ((e_torque * (data_.num_poles / 2)) - (data_.damping * state_.rotor_omega) -
					   state_.loadTorque);

	if (std::abs(m_torque) < state_.friction)
	{
		m_torque = 0;
	} else if (m_torque > 0 && std::abs(m_torque) >= state_.friction)
	{
		m_torque -= std::copysign(state_.friction, m_torque);
	}

	MODM_LOG_DEBUG << "Torque " << e_torque << " " << m_torque << modm::endl;

	state_.rotor_omega_dot = m_torque / data_.inertia;
	state_.rotor_omega += state_.rotor_omega_dot / timestep;
	state_.rotor_theta += state_.rotor_omega / timestep;
	state_.rotor_theta = angleMod(state_.rotor_theta);

	MODM_LOG_DEBUG << "Rotor " << state_.rotor_omega_dot << " " << state_.rotor_omega << " "
				   << state_.rotor_theta << modm::endl;

	updateVoltages();

	MODM_LOG_DEBUG << "V " << state_.phase_v(0) << " " << state_.phase_v(1) << " "
				   << state_.phase_v(2) << modm::endl;

	state_.phase_i = (state_.phase_v - data_.stator_phase_r * state_.phase_i - backEMF -
					  Eigen::Vector3d::Constant(state_.star_v)) /
					 (data_.stator_phase_l - data_.mutual_inductance);
	updateHallPort();
}

double
MotorSimulation::maxCurrent()
{
	return state_.phase_i.cwiseAbs().maxCoeff();
}

void
MotorSimulation::updateHallPort()
{
	const auto index =
		((unsigned int)std::round(angleMod(state_.rotor_theta) * 6 / (2 * M_PI))) % 6;

	Pin<0>::set(index == 5 || index == 0 || index == 1);
	Pin<1>::set(index == 1 || index == 2 || index == 3);
	Pin<2>::set(index == 3 || index == 4 || index == 5);
}

}  // namespace sim