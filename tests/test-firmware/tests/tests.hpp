#pragma once
#include <array>
#include <functional>
#include "motor_info.hpp"

#include "../motor.hpp"

bool
test_zero_current(Motor &);

bool
test_run_forwards(Motor &);

static inline std::array<std::function<bool(Motor &)>, 2> tests = {
	&test_zero_current,
	&test_run_forwards,
};
