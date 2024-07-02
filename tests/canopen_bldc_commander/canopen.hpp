#pragma once
#include "handlers.hpp"

#include <modm/platform/can/socketcan.hpp>

#include <modm-canopen/master/canopen_master.hpp>
#include <modm-canopen/master/sdo_client.hpp>
#include <modm-canopen/generated/micro-motor_od.hpp>

using modm_canopen::CanopenMaster;
using modm_canopen::CanopenNode;
using modm_canopen::generated::micromotor_OD;

#ifdef HOSTED
constexpr char canDevice[] = "vcan0";
constexpr uint8_t motorId = 10;  // Keep consistent with firmware
#else
constexpr char canDevice[] = "can0";
constexpr uint8_t motorId = 24;  // Keep consistent with firmware
#endif

static inline modm::platform::SocketCan can;

static inline auto sendMessage = [](const modm::can::Message& msg) { return can.sendMessage(msg); };

using MotorNode = CanopenNode<micromotor_OD, Test>;
using Master = CanopenMaster<MotorNode>;
using SdoClient = modm_canopen::SdoClient<Master>;