#pragma once
#include <cstdlib>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

using modm_canopen::cia402::OperatingMode;

struct CommandSendInfo
{
	modm_canopen::cia402::StateCommandNames name;
	OperatingMode mode;
	size_t time;
	void (*custom)(){nullptr};
};
