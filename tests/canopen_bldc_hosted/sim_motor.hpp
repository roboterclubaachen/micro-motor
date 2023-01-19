#pragma once
#include <limits>
#include <cstdint>

class MotorSimulation
{
private:
	static constexpr float pi = 3.14f;

	// rotorPosition_ value range: [-pi,pi)
	double rotorPosition_{0.0};

	// rotorVelocity_ unit: 1/s
	double rotorVelocity_{0.0};

	// rotorAcceleration_ unit: 1/s^2
	double rotorAcceleration_{0.0};

	// electricResistance_ unit: mOhms
	double electricResistance_mOhm{1000.0};

	// motorSpeedConstant_ unit: 1/(V*s)
	double motorSpeedConstant_1_Vs{0.1};

	// motorTorqueConstant_ unit: A/(N*m)
	double motorTorqueConstant_A_Nm{0.2};

	// minimumCurrent_ unit: mA
	double minimumCurrent_mA{100.0};

	// inputVoltage_ unit: mV
	double inputVoltage_mV{0.0};

	// shaftInertia_ unit: kg*m^2
	double shaftInertia_kgm2{1000.0};

public:
	MotorSimulation() = default;
	inline void
	setElectricResistance(float value_mOhm)
	{
		electricResistance_mOhm = value_mOhm;
	}
	inline void
	setMotorSpeedConstant(float value_1_Vs)
	{
		motorSpeedConstant_1_Vs = value_1_Vs;
	}
	inline void
	setMotorTorqueConstant(float value_A_Nm)
	{
		motorTorqueConstant_A_Nm = value_A_Nm;
	}
	inline void
	setMinimumCurrent(float value_mA)
	{
		minimumCurrent_mA = value_mA;
	}
	inline void
	setInputVoltage(float value_mV)
	{
		inputVoltage_mV = value_mV;
	}
	inline void
	setShaftInertia(float value_kgm2)
	{
		shaftInertia_kgm2 = value_kgm2;
	}

	void
	update(float timestep_s);

	uint8_t
	hall();

	void
	setInputVoltageInt(int16_t voltage);

	inline int32_t
	rawPosition()
	{
		return (int32_t)((rotorPosition_ / pi) * std::numeric_limits<int32_t>::max());
	}
	inline float
	position()
	{
		return rotorPosition_;
	}
	inline float
	velocity()
	{
		return rotorVelocity_;
	}
	inline float
	acceleration()
	{
		return rotorAcceleration_;
	}
};