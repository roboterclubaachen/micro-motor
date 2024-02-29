#pragma once
#include "canopen_callbacks.hpp"

#include <cassert>

constexpr uint8_t motorId = 12;

using modm_canopen::CanopenMaster;
using modm_canopen::CanopenNode;
using modm_canopen::cia402::OperatingMode;
using modm_canopen::cia402::State;
using modm_canopen::cia402::StateCommandNames;
using modm_canopen::cia402::StateCommands;
using modm_canopen::generated::micromotor_OD;

using MotorNode = CanopenNode<micromotor_OD, CanopenCallbacks>;
using Master = CanopenMaster<MotorNode>;
using SdoClient = modm_canopen::SdoClient<Master>;

template<typename MessageCallback>
void
updatePIDs(MessageCallback&& sendMessage)
{
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kP, state_.vPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kI, state_.vPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kD, state_.vPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kP, state_.pPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kI, state_.pPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kD, state_.pPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kP, state_.cPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kI, state_.cPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, CurrentObjects::CurrentPID_kD, state_.cPID_kD,
							std::forward<MessageCallback>(sendMessage));
}

template<typename MessageCallback>
void
setUnits(MessageCallback&& sendMessage)
{

	constexpr auto ticks_per_rev = 42;
	constexpr auto seconds_per_minute = 60;

	// Convert Ticks to rotations
	SdoClient::requestWrite(motorId, StateObjects::PositionFactorDivisor, ticks_per_rev,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, StateObjects::PositionFactorNumerator, 1,
							std::forward<MessageCallback>(sendMessage));

	// Convert ticks per second to rev/min
	SdoClient::requestWrite(motorId, StateObjects::VelocityFactorDivisor, ticks_per_rev,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, StateObjects::VelocityFactorNumerator, seconds_per_minute,
							std::forward<MessageCallback>(sendMessage));
}

template<typename MessageCallback>
void
configure(MessageCallback&& sendMessage)
{
	MotorNode::ReceivePdo_t statusRpdoMotor{};
	statusRpdoMotor.setInactive();
	assert(statusRpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::StatusWord, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(1, modm_canopen::PdoMapping{StateObjects::OutputPWM, 16}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMapping(
			   2, modm_canopen::PdoMapping{StateObjects::ModeOfOperationDisplay, 8}) ==
		   SdoErrorCode::NoError);
	assert(statusRpdoMotor.setMappingCount(3) == SdoErrorCode::NoError);
	assert(statusRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 0, statusRpdoMotor);
	Master::configureRemoteTPDO(motorId, 0, statusRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t velocityRpdoMotor{};
	velocityRpdoMotor.setInactive();
	assert(velocityRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{StateObjects::VelocityActualValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{VelocityObjects::VelocityDemandValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 1, velocityRpdoMotor);
	Master::configureRemoteTPDO(motorId, 1, velocityRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t currentRpdoMotor{};
	currentRpdoMotor.setInactive();
	assert(currentRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{CurrentObjects::FilteredActualCurrent, 32}) ==
		   SdoErrorCode::NoError);
	assert(currentRpdoMotor.setMapping(1, modm_canopen::PdoMapping{CurrentObjects::CommandedCurrent,
																   32}) == SdoErrorCode::NoError);
	assert(currentRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(currentRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 2, currentRpdoMotor);
	Master::configureRemoteTPDO(motorId, 2, currentRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::ReceivePdo_t positionRpdoMotor{};
	positionRpdoMotor.setInactive();
	assert(positionRpdoMotor.setMapping(
			   0, modm_canopen::PdoMapping{StateObjects::PositionActualValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(positionRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{PositionObjects::PositionDemandValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(positionRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(positionRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 3, positionRpdoMotor);
	Master::configureRemoteTPDO(motorId, 3, positionRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::ControlWord,
																   16}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{StateObjects::ModeOfOperation,
																   8}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 0, commandTpdoMotor);
	Master::configureRemoteRPDO(motorId, 0, commandTpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t targetTpdoMotor{};
	targetTpdoMotor.setInactive();
	assert(targetTpdoMotor.setMapping(0, modm_canopen::PdoMapping{CurrentObjects::TargetCurrent,
																  32}) == SdoErrorCode::NoError);
	assert(targetTpdoMotor.setMapping(1, modm_canopen::PdoMapping{VelocityObjects::TargetVelocity,
																  32}) == SdoErrorCode::NoError);
	assert(targetTpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(targetTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 1, targetTpdoMotor);
	Master::configureRemoteRPDO(motorId, 1, targetTpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	// NO TARGET POSITION MAPPING! Setting manually.
	SdoClient::requestWrite(motorId, PositionObjects::TargetPosition, state_.targetPosition,
							std::forward<MessageCallback>(sendMessage));

	// Update PIDs
	updatePIDs(std::forward<MessageCallback>(sendMessage));

	// Configure Motor Units
	setUnits(std::forward<MessageCallback>(sendMessage));
}

inline bool
makeReady()
{
	if (state_.stateMachine_.state() == State::OperationEnabled) return true;

	const auto oldControl = state_.control_.value();
	switch (state_.stateMachine_.state())
	{
		case State::ReadyToSwitchOn:
			state_.control_.apply(StateCommands[(uint8_t)StateCommandNames::SwitchOn].cmd);
			break;
		case State::SwitchOnDisabled:
			state_.control_.apply(StateCommands[(uint8_t)StateCommandNames::Shutdown].cmd);
			break;
		case State::SwitchedOn:
			state_.control_.apply(StateCommands[(uint8_t)StateCommandNames::EnableOperation].cmd);
			break;
		default:
			state_.control_.apply(StateCommands[(uint8_t)StateCommandNames::Shutdown].cmd);
			break;
	}
	if (oldControl != state_.control_.value())
	{
		MODM_LOG_INFO << "Sending Command..." << modm::endl;
		MODM_LOG_INFO << modm_canopen::cia402::stateToString(state_.stateMachine_.state())
					  << modm::endl;
	}
	Master::setValueChanged(StateObjects::ControlWord);
	return false;
}