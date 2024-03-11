#pragma once
#include "canopen.hpp"
#include "state.hpp"

template<typename MessageCallback>
void
setPDOs(MessageCallback&& sendMessage)
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

	MotorNode::ReceivePdo_t infoRpdoMotor{};
	infoRpdoMotor.setInactive();
	assert(infoRpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::VelocityActualValue,
																32}) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{VelocityObjects::VelocityDemandValue, 32}) ==
		   SdoErrorCode::NoError);
	assert(infoRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(infoRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 1, infoRpdoMotor);
	Master::configureRemoteTPDO(motorId, 1, infoRpdoMotor, 100,
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

	MotorNode::ReceivePdo_t velocityRpdoMotor{};
	velocityRpdoMotor.setInactive();
	assert(velocityRpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::OrientedCurrent,
																	32}) == SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMapping(
			   1, modm_canopen::PdoMapping{StateObjects::OrientedCurrentAngleDiff, 32}) ==
		   SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setMappingCount(2) == SdoErrorCode::NoError);
	assert(velocityRpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setRPDO(motorId, 3, velocityRpdoMotor);
	Master::configureRemoteTPDO(motorId, 3, velocityRpdoMotor, 100,
								std::forward<MessageCallback>(sendMessage));

	MotorNode::TransmitPdo_t commandTpdoMotor{};
	commandTpdoMotor.setInactive();
	assert(commandTpdoMotor.setMapping(0, modm_canopen::PdoMapping{StateObjects::ControlWord,
																   16}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(1, modm_canopen::PdoMapping{StateObjects::ModeOfOperation,
																   8}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMapping(2, modm_canopen::PdoMapping{VelocityObjects::TargetVelocity,
																   32}) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setMappingCount(3) == SdoErrorCode::NoError);
	assert(commandTpdoMotor.setActive() == SdoErrorCode::NoError);
	Master::setTPDO(motorId, 0, commandTpdoMotor);
	Master::configureRemoteRPDO(motorId, 0, commandTpdoMotor,
								std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, CurrentObjects::TargetCurrent, state.targetCurrent,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::TargetVelocity, state.targetSpeed,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::TargetPosition, state.targetPosition,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PWMObjects::PWMCommand, state.commandedPWM,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kP, vPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kI, vPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, VelocityObjects::VelocityPID_kD, vPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kP, pPID_kP,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kI, pPID_kI,
							std::forward<MessageCallback>(sendMessage));
	SdoClient::requestWrite(motorId, PositionObjects::PositionPID_kD, pPID_kD,
							std::forward<MessageCallback>(sendMessage));

	SdoClient::requestWrite(motorId, CurrentObjects::ShouldInvert,(uint8_t) invert,
							std::forward<MessageCallback>(sendMessage));
}