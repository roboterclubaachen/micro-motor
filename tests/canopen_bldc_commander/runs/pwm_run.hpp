
#pragma once
#include <cstdlib>
#include <modm-canopen/cia402/state_commands.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>

#include "../command_send_info.hpp"
#include "../state.hpp"
#include "../canopen.hpp"

#ifdef PWM_CMDS
#define CMDLIST
size_t maxTime = 10100;

constexpr std::array sendCommands{
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::Shutdown},
					.mode{OperatingMode::Voltage},
					.time{10},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::SwitchOn},
					.mode{OperatingMode::Voltage},
					.time{20},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::EnableOperation},
					.mode{OperatingMode::Voltage},
					.time{30},
					.custom{nullptr}},
	CommandSendInfo{.name{modm_canopen::cia402::StateCommandNames::DisableVoltage},
					.mode{OperatingMode::Voltage},
					.time{10030},
					.custom{nullptr}},
};
#endif