#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <librobots2/motor/bldc_motor_block_commutation.hpp>
#include <micro-motor/hardware.hpp>
#include <modm/processing/timer.hpp>
#include <modm/math/filter/pid.hpp>

enum class ControlMode : int8_t
{
	Disabled = 0,
	Voltage = -1,
	Velocity = 3
};

using Pid = modm::Pid<float>;

class Motor
{
private:
	librobots2::motor::BldcMotorBlockCommutation<Board::Motor> motor_;
	bool enabled_{false};
	ControlMode mode_{ControlMode::Voltage};
	int32_t commandedVelocity_{};
	int32_t actualVelocity_{};
	int32_t actualPosition_{};
	int16_t commandedVoltage_{};
	int16_t outputVoltage_{};
	modm::PeriodicTimer controlTimer_{std::chrono::milliseconds{10}};
	Pid velocityPid_;
	uint_fast8_t commutationOffset_;
	uint_fast8_t lastHallState_{};
	int32_t lastPosition_{};

	using Hall = librobots2::motor::HallPermutations<Board::Motor::HallPort>;

	uint_fast8_t readHall()
	{
		const auto& HallStates = librobots2::motor::block_commutation::SequenceLut;
		return HallStates[Hall::read(commutationOffset_) & 0b111];
	}

	void updateVelocity();
	void updatePosition();

public:
	Motor(uint_fast8_t commutationOffset, const Pid::Parameter& parameters)
		: motor_{commutationOffset}, commutationOffset_{commutationOffset}
	{
		velocityPid_.setParameter(parameters);
	}

	void initializeHall() { lastHallState_ = readHall(); }

	ControlMode mode() { return mode_; }
	void setMode(ControlMode mode) { mode_ = mode; }

	bool isEnabled() const { return enabled_; }
	void setEnabled(bool enabled)
	{
		enabled_ = enabled;
		if (!enabled) {
			motor_.disable();
		}
	}

	int32_t commandedVelocity() const { return commandedVelocity_; }
	void setCommandedVelocity(int32_t velocity) { commandedVelocity_ = velocity; }

	int16_t commandedVoltage() const { return commandedVoltage_; }
	void setCommandedVoltage(int16_t voltage) { commandedVoltage_ = voltage; }

	int32_t velocity() const { return actualVelocity_; }
	int32_t position() const { return actualPosition_; }
	int16_t outputVoltage() const { return outputVoltage_; }

	bool update();
};

const Pid::Parameter controllerParameters {
	// TODO
	1.0f, // kp
	0.0f, // ki
	0.0f, // kd
	0.0f, // max error sum
	(float) std::numeric_limits<int32_t>::max() // max output
};

constexpr uint8_t motor0CommutationOffset{0};
inline Motor Motor0{motor0CommutationOffset, controllerParameters};
