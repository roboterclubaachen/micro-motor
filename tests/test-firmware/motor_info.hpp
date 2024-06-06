#pragma once
#include <cstdint>

struct MotorInfo
{
	uint8_t commutationOffset = 1;
	bool reversed = false;
};

static inline MotorInfo motorInfo0;