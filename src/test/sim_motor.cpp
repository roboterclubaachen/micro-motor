#include "sim_motor.hpp"
#include <limits>

void
MotorSimulation::update(float timestep_s)
{

	double inputCurrent_mA = 1000 * (inputVoltage_mV) / electricResistance_mOhm;

	if (inputCurrent_mA > 0.0)
	{
		inputCurrent_mA -= minimumCurrent_mA;
		if (inputCurrent_mA < 0.0) inputCurrent_mA = 0.0;
	} else if (inputCurrent_mA < 0.0)
	{
		inputCurrent_mA += minimumCurrent_mA;
		if (inputCurrent_mA > 0.0) inputCurrent_mA = 0.0;
	}

	double backEMF_mV = 1000.0 * rotorVelocity_ / motorSpeedConstant_1_Vs;
	double backEMFCurrent_ma = 1000 * (backEMF_mV) / electricResistance_mOhm;

	double torque_Nm = (inputCurrent_mA - backEMFCurrent_ma) / (1000.0 * motorTorqueConstant_A_Nm);

	rotorAcceleration_ = torque_Nm / shaftInertia_kgm2;
	rotorVelocity_ += rotorAcceleration_ * timestep_s;
	rotorPosition_ += rotorVelocity_ * timestep_s;
	while (rotorPosition_ > pi) rotorPosition_ -= 2 * pi;
	while (rotorPosition_ < -pi) rotorPosition_ += 2 * pi;
}

uint8_t
MotorSimulation::hall()
{
	return (uint8_t)(6 * (rotorPosition_ + pi) / (2 * pi));
}

void
MotorSimulation::setInputVoltageInt(int16_t voltage)
{
	constexpr float maxVoltage_mV = 12000.0f;
	float outputVoltage_mV =
		((float)voltage / (float)std::numeric_limits<int16_t>::max()) * maxVoltage_mV;
	setInputVoltage(outputVoltage_mV);
}