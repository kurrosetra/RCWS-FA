/*
 Ingenia_SerialServoDrive.h - Library for control Ingenia Servo Drives.
 */

#ifndef INGENIA_SERIALSERVODRIVE_h
#define INGENIA_SERIALSERVODRIVE_h

#include <stdbool.h>
#include "stm_hal_serial.h"

#define MAX_MSG_LEN   40

typedef struct
{
	TSerial * serial;
	bool _isBinaryEnabled;
	char serial_buf[MAX_MSG_LEN];
	uint8_t _u8Node;
	bool _isInitialAngleDeterminationProcessFinished;
} TServo;

//! The enumeration of Drive Modes
typedef enum
{
	DRIVE_MODE_PROFILE_POSITION = 1,            //!< Profile position mode
	DRIVE_MODE_PROFILE_VELOCITY = 3,            //!< Profile velocity mode
	DRIVE_MODE_PROFILE_TORQUE = 4,              //!< Profile torque mode
	//DRIVE_MODE_VELOCITY = 2,                  //!< Velocity mode
	DRIVE_MODE_HOMING = 6,                      //!< Homing mode
	DRIVE_MODE_INTERPOLATED_POSITION = 7,       //!< Interpolated position mode
	DRIVE_MODE_CYCLIC_SYNC_POSITION = 8,        //!< Cyclic sync position mode
	DRIVE_MODE_CYCLIC_SYNC_VELOCITY = 9,        //!< Cyclic sync velocity mode
	DRIVE_MODE_CYCLIC_SYNC_TORQUE = 10,         //!< Cyclic sync torque mode
	DRIVE_MODE_OPEN_LOOP_SCALAR = -1,           //!< Open loop scalar mode
	DRIVE_MODE_OPEN_LOOP_VECTOR = -2            //!< Open loop vector mode
} EDriveModes;

/* PUBLIC */

uint32_t Ingenia_readReg(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex);
uint32_t Ingenia_readAdv(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, bool * bIsValid);

void Ingenia_write_u64(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint64_t value);
void Ingenia_write_i64(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int64_t value);
void Ingenia_write_u32(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint32_t value);
void Ingenia_write_i32(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int32_t value);
void Ingenia_write_u16(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint16_t value);
void Ingenia_write_i16(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int16_t value);
void Ingenia_write_u8(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint8_t value);
void Ingenia_write_i8(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int8_t value);

// Binary enable/disable functions
void Ingenia_enableBinary(TServo * servo);
void Ingenia_disableBinary(TServo * servo);

// Motor enable/disable functions
void Ingenia_enableMotor(TServo * servo);
void Ingenia_disableMotor(TServo * servo);

// Homing
void Ingenia_doHoming(TServo * servo, int8_t i8HomingMethod);

// Motion funcions
void Ingenia_setModeOfOperation(TServo * servo, EDriveModes driverMode);

void Ingenia_setTargetVelocity(TServo * servo, int32_t value);
void Ingenia_setTargetTorque(TServo * servo, int16_t value);
void Ingenia_setTargetPosition(TServo * servo, int32_t value);
void Ingenia_setTargetPositionAdv(TServo * servo, int32_t value, bool isImmediate, bool isRelative,
bool isHaltEnabled);

int32_t Ingenia_getActualPosition(TServo * servo);
int32_t Ingenia_getActualVelocity(TServo * servo);
int16_t Ingenia_getActualTorque(TServo * servo);

// Statusword functions
uint16_t Ingenia_getStatusword(TServo * servo);
bool Ingenia_statuswordIsReadyToSwitchOn(TServo * servo);
bool Ingenia_statuswordIsSwitchedOn(TServo * servo);
bool Ingenia_statuswordIsOperationEnabled(TServo * servo);
bool Ingenia_statuswordIsFault(TServo * servo);
bool Ingenia_statuswordIsVoltageEnabled(TServo * servo);
bool Ingenia_statuswordIsQuickStop(TServo * servo);
bool Ingenia_statuswordIsSwitchOnDisabled(TServo * servo);
bool Ingenia_statuswordIsWarning(TServo * servo);
bool Ingenia_statuswordIsRemote(TServo * servo);
bool Ingenia_statuswordIsTargetReached(TServo * servo);
bool Ingenia_statuswordIsInternalLimitActive(TServo * servo);
bool Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(TServo * servo);

// Homing status functions
bool Ingenia_homingStatusIsInProgress(TServo * servo);
bool Ingenia_homingStatusIsError(TServo * servo);
bool Ingenia_homingStatusIsSuccess(TServo * servo);
bool Ingenia_homingStatusIsAttained(TServo * servo);
bool Ingenia_homingStatusIsInterrupted(TServo * servo);
bool Ingenia_homingStatusNotStarted(TServo * servo);

/* PRIVATE */

void Ingenia_write(TServo *servo, uint32_t u32Index, uint8_t u8SubIndex, uint32_t u32ObjSize,
		uint64_t value);

// Types of access to registers
typedef enum
{
	READ = 0,               //!< Read access to register (read data)
	WRITE               //!< Write access to register (send data)
} ERegAccess;

// Multiple access for a region of memory of 64 bits
typedef union
{
	int64_t i64;            //!< As one 64 integer
	uint64_t u64;            //!< As one 64 unsigned integer
	int32_t i32[2];            //!< As two 32 integer
	uint32_t u32[2];            //!< As two 32 unsigned integer
	int16_t i16[4];            //!< As four 16 integers
	uint16_t u16[4];            //!< As four 16 unsigned integers
	int8_t i8[8];            //!< As eight 8 integers
	uint8_t u8[8];            //!< As eight 8 unsigned integers
	unsigned char b8[8];            //!< As eight 8 bytes
} t_int64;

// Message data structure
typedef struct
{
	unsigned char b8ID;             //!< ID of motion controller
	uint32_t u32Register;             //!< Register to access
	ERegAccess eRegAccess;             //!< Acces type (read or write)
	unsigned char b8DataLength;             //!< Data length for write access
	t_int64 ti64Data;             //!< Data
} TMessage, *PTMessage;

int Ingenia_serial_read(TServo * servo, unsigned long timeout);
void Ingenia_serial_write(TServo * servo, char* strMessage, size_t sLength);
bool Ingenia_encodeMsg(TMessage *pMsg, char* strMessage, size_t *pubLenght, bool isBinary);
bool Ingenia_decodeMsg(const char* strMessage, TMessage *pMsg, bool isBinary);

//! State Machine Commands
typedef enum
{
	COMMAND_SHUTDOWN,                           //!< Shutdown command
	COMMAND_SWITCH_ON,                           //!< Switch on command
	COMMAND_SWITCH_ON_AND_ENABLE_OPERATION,                //!< Switch on + Enable operation command
	COMMAND_DISABLE_VOLTAGE,                           //!< Disable voltage command
	COMMAND_QUICK_STOP,                           //!< Quick stop command
	COMMAND_DISABLE_OPERATION,                           //!< Disable operation command
	COMMAND_ENABLE_OPERATION,                           //!< Enable operation command
	COMMAND_FAULT_RESET                           //!< Fault reset command
} EStateMachineCommand;

//! The enumeration of State Machine Status
typedef enum
{
	STATUS_NOT_READY_TO_SWITCH_ON,      //!< Not ready to switch ON
	STATUS_SWITCH_ON_DISABLED,      //!< Switch ON disabled
	STATUS_READY_TO_SWITCH_ON,      //!< Ready to switch ON
	STATUS_SWITCH_ON,      //!< Switch ON status
	STATUS_OPERATION_ENABLED,      //!< Operation enabled
	STATUS_QUICK_STOP_ACTIVE,      //!< Quick stop active
	STATUS_FAULT_REACTION_ACTIVE,      //!< Fault reaction active
	STATUS_FAULT,      //!< Fault status
	STATUS_UNKNOWN      //!< Unknown status
} EStateMachineStatus;

// State machine
//
//void Ingenia_sendStateMachineCommand(TServo *servo, const EStateMachineCommand eStateMachineCommand);
//void Ingenia_setStateMachineStatus(TServo *servo, const EStateMachineStatus eNewStateMachineStatus);
//EStateMachineStatus Ingenia_decodeStatusWord(TServo *servo, uint16_t u16StatusWord);
void Ingenia_getStateMachineStatus(TServo *servo, EStateMachineStatus* peStateMachineStatus);
//void Ingenia_goToStatusFrom(TServo *servo, const EStateMachineStatus eCurrentStateMachineStatus,
//		const EStateMachineStatus eDestinationStateMachineStatus);
//bool Ingenia_verifyStatusIsReached(TServo *servo, const EStateMachineStatus eNextStateMachineStatus);

#endif

