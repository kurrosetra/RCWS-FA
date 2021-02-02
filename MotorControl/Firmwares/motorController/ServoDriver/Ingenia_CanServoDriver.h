/*
 * Ingenia_CanServoDriver.h
 *
 *  Created on: Feb 14, 2020
 *      Author: miftakur
 */

#ifndef INGENIA_CANSERVODRIVER_H_
#define INGENIA_CANSERVODRIVER_H_

#include "can_buffer.h"

typedef struct
{
	CAN_Buffer_t bufTSDO;
	CAN_Buffer_t bufNMT;
	CAN_Buffer_t bufTPDO;
	CAN_Buffer_t bufEMER;
} Ingenia_Buffer_t;

typedef struct
{
	Ingenia_Buffer_t buffer;
	uint8_t _isRegistered;
	uint8_t _u8Node;
	uint8_t _isInitialAngleDeterminationProcessFinished;
	uint16_t statusword;
	int32_t posActual;
	int32_t veloActual;
} Servo_t;

typedef enum
{
	COB_NMT_SERVICE = 0,
	COB_EMERGENCY = 0x80,
	COB_TPDO1 = 0x180,
	COB_TPDO2 = 0x280,
	COB_TPDO3 = 0x380,
	COB_TPDO4 = 0x480,
	COB_RPDO1 = 0x200,
	COB_RPDO2 = 0x300,
	COB_RPDO3 = 0x400,
	COB_RPDO4 = 0x500,
	COB_TSDO = 0x580,
	COB_RSDO = 0x600,
	COB_NMT_CTRL = 0x700
} COB_ID_e;

HAL_StatusTypeDef Ingenia_begin(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef Ingenia_init(Servo_t *servo, uint8_t node);
HAL_StatusTypeDef Ingenia_node_remove(Servo_t *servo);
HAL_StatusTypeDef Ingenia_buffer_available(CAN_Buffer_t *buf, CAN_Data_t *data);
HAL_StatusTypeDef Ingenia_IRQHandler();
void Ingenia_emer_callback(CAN_Buffer_t *buffer);
void Ingenia_nmt_callback(CAN_Buffer_t *buffer);
void Ingenia_tpdo_callback(CAN_Buffer_t *buffer);
void Ingenia_tsdo_callback(CAN_Buffer_t *buffer);

//! State Machine Commands
typedef enum
{
	COMMAND_SHUTDOWN,						//!< Shutdown command
	COMMAND_SWITCH_ON,						//!< Switch on command
	COMMAND_SWITCH_ON_AND_ENABLE_OPERATION,  //!< Switch on + Enable operation command
	COMMAND_DISABLE_VOLTAGE,				//!< Disable voltage command
	COMMAND_QUICK_STOP,						//!< Quick stop command
	COMMAND_DISABLE_OPERATION,				//!< Disable operation command
	COMMAND_ENABLE_OPERATION,				//!< Enable operation command
	COMMAND_FAULT_RESET						//!< Fault reset command
} StateMachineCommand_e;

//! The enumeration of State Machine Status
typedef enum
{
	STATUS_NOT_READY_TO_SWITCH_ON,	//!< Not ready to switch ON
	STATUS_SWITCH_ON_DISABLED,		//!< Switch ON disabled
	STATUS_READY_TO_SWITCH_ON,		//!< Ready to switch ON
	STATUS_SWITCH_ON,				//!< Switch ON status
	STATUS_OPERATION_ENABLED,		//!< Operation enabled
	STATUS_QUICK_STOP_ACTIVE,		//!< Quick stop active
	STATUS_FAULT_REACTION_ACTIVE,	//!< Fault reaction active
	STATUS_FAULT,					//!< Fault status
	STATUS_UNKNOWN					//!< Unknown status
} StateMachineStatus_e;

//! The enumeration of Drive Modes
typedef enum
{
	DRIVE_MODE_PROFILE_POSITION = 1,            //!< Profile position mode
	DRIVE_MODE_PROFILE_VELOCITY = 3,            //!< Profile velocity mode
	DRIVE_MODE_PROFILE_TORQUE = 4,              //!< Profile torque mode
	DRIVE_MODE_VELOCITY = 2,                    //!< Velocity mode
	DRIVE_MODE_HOMING = 6,                      //!< Homing mode
	DRIVE_MODE_INTERPOLATED_POSITION = 7,       //!< Interpolated position mode
	DRIVE_MODE_CYCLIC_SYNC_POSITION = 8,        //!< Cyclic sync position mode
	DRIVE_MODE_CYCLIC_SYNC_VELOCITY = 9,        //!< Cyclic sync velocity mode
	DRIVE_MODE_CYCLIC_SYNC_TORQUE = 10,         //!< Cyclic sync torque mode
	DRIVE_MODE_OPEN_LOOP_SCALAR = -1,           //!< Open loop scalar mode
	DRIVE_MODE_OPEN_LOOP_VECTOR = -2            //!< Open loop vector mode
} DriveModes_e;

//! The enumeration of NMT Modes
typedef enum
{
	NMT_START_REMOTE_NODE = 1,
	NMT_STOP_REMOTE_NODE = 2,
	NMT_ENTER_PRE_OPS = 0x80,
	NMT_RESET_NODE = 0x81,
	NMT_RESET_COMM = 0x82
} NmtModes_e;

void Ingenia_write_nmt(Servo_t *servo, NmtModes_e mode);

void Ingenia_write_rpdo(Servo_t *servo, uint32_t cob_rpdo, uint8_t* data, uint8_t len);

uint32_t Ingenia_read_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t *bIsValid);
uint32_t Ingenia_read_reg_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex);

HAL_StatusTypeDef Ingenia_write_sdo_u32(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint32_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i32(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int32_t value);
HAL_StatusTypeDef Ingenia_write_sdo_u16(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint16_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i16(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int16_t value);
HAL_StatusTypeDef Ingenia_write_sdo_u8(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint8_t value);
HAL_StatusTypeDef Ingenia_write_sdo_i8(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int8_t value);

void Ingenia_gotoStatus(Servo_t *servo, const StateMachineStatus_e eDestinationStateMachineStatus);

// Motor enable/disable functions
void Ingenia_enableMotor(Servo_t * servo);
void Ingenia_disableMotor(Servo_t * servo);

// Homing
void Ingenia_doHoming(Servo_t * servo, int8_t i8HomingMethod);

// Motion funcions
void Ingenia_setModeOfOperation(Servo_t * servo, DriveModes_e driverMode);

void Ingenia_setTargetVelocity(Servo_t * servo, int32_t value);
void Ingenia_setTargetTorque(Servo_t * servo, int16_t value);
void Ingenia_setTargetPosition(Servo_t * servo, int32_t value);
void Ingenia_setTargetPositionAdv(Servo_t * servo, int32_t value, uint8_t isImmediate,
		uint8_t isRelative, uint8_t isHaltEnabled);
void Ingenia_setTargetPositionVelocity(Servo_t * servo, int32_t pos, uint32_t velo,
		uint8_t isImmediate, uint8_t isRelative, uint8_t isHaltEnabled);

int32_t Ingenia_getActualPosition(Servo_t * servo);
int32_t Ingenia_getActualVelocity(Servo_t * servo);
int16_t Ingenia_getActualTorque(Servo_t * servo);

// Statusword functions
uint16_t Ingenia_getStatusword(Servo_t * servo);
uint8_t Ingenia_statuswordIsReadyToSwitchOn(Servo_t * servo);
uint8_t Ingenia_statuswordIsSwitchedOn(Servo_t * servo);
uint8_t Ingenia_statuswordIsOperationEnabled(Servo_t * servo);
uint8_t Ingenia_statuswordIsFault(Servo_t * servo);
uint8_t Ingenia_statuswordIsVoltageEnabled(Servo_t * servo);
uint8_t Ingenia_statuswordIsQuickStop(Servo_t * servo);
uint8_t Ingenia_statuswordIsSwitchOnDisabled(Servo_t * servo);
uint8_t Ingenia_statuswordIsWarning(Servo_t * servo);
uint8_t Ingenia_statuswordIsRemote(Servo_t * servo);
uint8_t Ingenia_statuswordIsTargetReached(Servo_t * servo);
uint8_t Ingenia_statuswordIsInternalLimitActive(Servo_t * servo);
uint8_t Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(Servo_t * servo);

// Homing status functions
uint8_t Ingenia_homingStatusIsInProgress(Servo_t * servo);
uint8_t Ingenia_homingStatusIsError(Servo_t * servo);
uint8_t Ingenia_homingStatusIsSuccess(Servo_t * servo);
uint8_t Ingenia_homingStatusIsAttained(Servo_t * servo);
uint8_t Ingenia_homingStatusIsInterrupted(Servo_t * servo);
uint8_t Ingenia_homingStatusNotStarted(Servo_t * servo);

#endif /* INGENIA_CANSERVODRIVER_H_ */
