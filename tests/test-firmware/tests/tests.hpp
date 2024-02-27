#pragma once
#include <array>
#include <functional>
#include "motor_info.hpp"

#include "../motor.hpp"

bool
test_zero_current(Motor &, const MotorInfo &);

bool
test_run_forwards(Motor &, const MotorInfo &);

bool
test_current_limit(Motor &, const MotorInfo &);

static inline std::array<std::function<bool(Motor &, const MotorInfo &)>, 3> tests = {
	&test_zero_current,
	&test_run_forwards,
	&test_current_limit,
};
