/*
 * Ingenia_SerialServoDrive.c
 * Ingenia_SerialServoDrive.cpp - Ingenia serial servo drives master library for STM32
 *
 *  Created on: Nov 19, 2018
 *      Author: miftakur
 */

#include "Ingenia_SerialServoDrive.h"
#include <stdlib.h>
#include <string.h>

#define RS232_CRCHAR  0x0D
#define MAX_NUM_MC    127     //!< Maximum number of posible motion controllers connected to a single channel

// State machine
//
void Ingenia_sendStateMachineCommand(TServo *servo, const EStateMachineCommand eStateMachineCommand);
void Ingenia_setStateMachineStatus(TServo *servo, const EStateMachineStatus eNewStateMachineStatus);
EStateMachineStatus Ingenia_decodeStatusWord(TServo *servo, uint16_t u16StatusWord);
//void Ingenia_getStateMachineStatus(TServo *servo, EStateMachineStatus* peStateMachineStatus);
void Ingenia_goToStatusFrom(TServo *servo, const EStateMachineStatus eCurrentStateMachineStatus,
		const EStateMachineStatus eDestinationStateMachineStatus);
void Ingenia_gotoStatus(TServo *servo, const EStateMachineStatus eDestinationStateMachineStatus);
bool Ingenia_verifyStatusIsReached(TServo *servo, const EStateMachineStatus eNextStateMachineStatus);

uint32_t Ingenia_readReg(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex)
{
	uint32_t u32Message;
	bool isValidRead = false;

	while (!isValidRead) {
		u32Message = Ingenia_readAdv(servo, u32Index, u8SubIndex, &isValidRead);
	}

	return u32Message;
}

uint32_t Ingenia_readAdv(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, bool * bIsValid)
{
	uint32_t u32Result = 0;
	TMessage tMessage, tMsgTem;
	PTMessage decodedMessage = &tMsgTem;
	tMessage.b8ID = servo->_u8Node;
	tMessage.u32Register = ((uint32_t) u8SubIndex << 16) | u32Index;
	tMessage.eRegAccess = READ;
	tMessage.b8DataLength = 0;

	size_t eMsgLen;
	char eMsgStr[MAX_MSG_LEN];
	Ingenia_encodeMsg(&tMessage, eMsgStr, &eMsgLen, servo->_isBinaryEnabled);
	uint32_t millis;

	// Flush port
	while (serial_available(servo->serial))
		serial_read(servo->serial);

	Ingenia_serial_write(servo, eMsgStr, eMsgLen);

	bool isValid = false;
	unsigned long long started_waiting_at = HAL_GetTick();
	while (!isValid) {
		millis = HAL_GetTick();
		// Check if timeout excedeed
		if (millis - started_waiting_at > 1000) {
			break;
		}

		if (Ingenia_serial_read(servo, 200) != 0) {
			Ingenia_decodeMsg((const char*) &servo->serial_buf, decodedMessage,
					servo->_isBinaryEnabled);
			if (decodedMessage->u32Register == tMessage.u32Register
					&& decodedMessage->eRegAccess == WRITE
					&& decodedMessage->b8ID == tMessage.b8ID) {
				isValid = true;
			}

			if (!isValid)
				Ingenia_serial_write(servo, eMsgStr, eMsgLen);
		}
	}

	if (isValid)
		u32Result = decodedMessage->ti64Data.u32[0];

	*bIsValid = isValid;

//	delete(eMsgStr);
//	delete(decodedMessage);
	return u32Result;
}

void Ingenia_write_u64(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint64_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 8, value);
}

void Ingenia_write_i64(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int64_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 8, value);
}
void Ingenia_write_u32(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint32_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 4, value);
}
void Ingenia_write_i32(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int32_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 4, value);
}
void Ingenia_write_u16(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint16_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 2, value);
}
void Ingenia_write_i16(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int16_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 2, value);
}
void Ingenia_write_u8(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, uint8_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 1, value);
}
void Ingenia_write_i8(TServo * servo, uint32_t u32Index, uint8_t u8SubIndex, int8_t value)
{
	Ingenia_write(servo, u32Index, u8SubIndex, 1, value);
}

void Ingenia_write(TServo *servo, uint32_t u32Index, uint8_t u8SubIndex, uint32_t u32ObjSize,
		uint64_t value)
{
	uint32_t u32Register;
	t_int64 i64DataValue;
	switch (u32ObjSize)
	{
	case 1:
		i64DataValue.i8[0] = (int8_t) value;
		break;
	case 2:
		i64DataValue.i16[0] = (int16_t) value;
		break;
	case 4:
		i64DataValue.i32[0] = (int32_t) value;
		break;
	case 8:
		i64DataValue.i64 = (int64_t) value;
		break;
	default:
		break;
	}

	u32Register = ((uint32_t) u8SubIndex << 16) | u32Index;

	// Prepare message to send
	TMessage tMessage;
	tMessage.b8ID = servo->_u8Node;
	tMessage.u32Register = u32Register;
	tMessage.eRegAccess = WRITE;
	tMessage.b8DataLength = u32ObjSize;
	tMessage.ti64Data = i64DataValue;

	// Send message thorugh interface
	size_t eMsgLen;
	char eMsgStr[MAX_MSG_LEN];
	Ingenia_encodeMsg(&tMessage, eMsgStr, &eMsgLen, servo->_isBinaryEnabled);
	Ingenia_serial_write(servo, eMsgStr, eMsgLen);
//	delete(eMsgStr);
}

// Binary Defines

#define MIN_BIN_FRAME_SIZE          13
#define NUMBER_OF_DATA_FIELD_SIZE    2
#define NUMBER_OF_DATA_HIGH         11
#define NUMBER_OF_DATA_LOW          12
#define MAX_DATA_FIELD_SIZE          8

#define DEFAULT_TIMEOUTS  1000

#define SYNC_BYTE_BIN       0x55        //!< synchronization byte Bin Mode
#define NUM_SYNC_BYTE       4           //!< Number of Synchronization bytes

int Ingenia_serial_read(TServo * servo, unsigned long timeout)
{
	uint16_t u16NumberOfData = 0;
	uint8_t u8EndMsgCounter = 0;
	uint8_t u8Counter = 0;
	bool isMessageCompleted = false;
	uint32_t started_waiting_at = HAL_GetTick();
	uint32_t millis = 0;

	while (!isMessageCompleted) {
		uint8_t b8RXBuff = 0;
		millis = HAL_GetTick();

		// Check if timeout excedeed
		if (millis - started_waiting_at > timeout) {
			break;
		}

		// Check if there are bytes waiting for read
		if (serial_available(servo->serial))
			b8RXBuff = (uint8_t) serial_read(servo->serial);
		else
			continue;

		if (!servo->_isBinaryEnabled) {
			if (!b8RXBuff)  // avoid null characters
			{
				continue;
			}
		}

		// if message lenght excedeed
		if (u8Counter >= MAX_MSG_LEN) {
			break;
		}

		servo->serial_buf[u8Counter] = b8RXBuff;

		if (servo->_isBinaryEnabled) {
			/* Fill buffer until it have the minimum expected size. Minimum size: 13 */
			/* ---------------------------------------------------------------------- */
			/*| Address | Function | MEI type | Prot. Control | Res. Filed | Node ID |*/
			/*|   1     |    1     |     1    |       1       |      1     |     1   |*/
			/* ---------------------------------------------------------------------- */
			/* -------------------------------------------------------------------    */
			/*| Index | Subindex | Starting Address | Number of data | Data | CRC |   */
			/*|   2   |    1     |         2        |        2       |   0  |  0  |   */
			/* -------------------------------------------------------------------    */
			/* Buffer is filled until Starting Address field at least         */
			// TODO: Add defines
			if (u8Counter > (MIN_BIN_FRAME_SIZE - NUMBER_OF_DATA_FIELD_SIZE)) {
				/* Save the high byte of the number of data field*/
				if (u8Counter == NUMBER_OF_DATA_HIGH)  // TODO: Add defines
				{
					u16NumberOfData = (uint16_t) (b8RXBuff << 8) & 0xFF00;
				}
				/* Save the low byte of the number of data field*/
				// TODO: Add defines
				else if (u8Counter == NUMBER_OF_DATA_LOW) {
					u16NumberOfData = (uint16_t) (b8RXBuff) & 0x00FF;
					if (u16NumberOfData < MAX_DATA_FIELD_SIZE + 1) {
						/* Add the CRC field size to the number of data to be received from
						 this point */
						/* if CRC enable
						 u16NumberOfData += 2;
						 */
					}
					/* if number of data is outside of the expeted value,
					 * the packet is droped. */
					else {
						u16NumberOfData = 0;
					}
				}
				else {
					/* if frame have all data, wait until four consecutive sync bytes
					 * are received */
					/* It considers that all data has been received when the frame size
					 * is equal to Minimum size + number of data inside data field + CRC
					 *  if it is  available */
					if (b8RXBuff == SYNC_BYTE_BIN
							&& (u8Counter >= (u16NumberOfData + (uint32_t) MIN_BIN_FRAME_SIZE)))  // Binary mode ends with 0x55
							{
						u8EndMsgCounter++;
						if (u8EndMsgCounter == NUM_SYNC_BYTE) {
							// Message reception completed
							// Decode message and write it into TMessage structure
							isMessageCompleted = true;
							continue;
						}
					}
					/* Fill buffer if there are more bytes in the data field*/
					else {
						u8EndMsgCounter = 0;
					}
				}
			}
		}
		else {
			if (b8RXBuff == RS232_CRCHAR) {
				servo->serial_buf[u8Counter] = 0;
				isMessageCompleted = true;
				continue;
			}
		}
		u8Counter++;
	}

	if (isMessageCompleted) {
		return u8Counter;
	}
	else
		return 0;
}
void Ingenia_serial_write(TServo * servo, char* strMessage, size_t sLength)
{
	serial_write_str(servo->serial, strMessage, sLength);
}

#define MCL_MSG_MAXDATALENGTH       8

bool Ingenia_encodeMsg(TMessage *pMsg, char* strMessage, size_t *pubLenght, bool isBinary)
{
	unsigned char b8NodeID;
	uint32_t u32Register;
	uint8_t u8DataLength;
	t_int64 ti64Data;
	uint8_t u8I;

	/* Verify parameters */
	if (pMsg == NULL || strMessage == NULL) {
		return false;
	}

	/* Start encoding message process */
	b8NodeID = pMsg->b8ID;
	u32Register = pMsg->u32Register;
	u8DataLength = pMsg->b8DataLength;

	if (isBinary) {
		/* CRC: uint16_t u16CRC; */
		uint8_t u8Index = 0, u8i;

		strMessage[u8Index++] = pMsg->b8ID; /* ModBus Address */
		strMessage[u8Index++] = 43; /* ModBus Function */
		strMessage[u8Index++] = 13; /* ModBus MEI */
		strMessage[u8Index++] = pMsg->eRegAccess; /* Protocol control */
		strMessage[u8Index++] = 0; /* reserved Field */
		strMessage[u8Index++] = pMsg->b8ID; /* Node ID */
		strMessage[u8Index++] = (u32Register >> 8) & 0xFF;
		strMessage[u8Index++] = (u32Register) & 0xFF;
		strMessage[u8Index++] = (u32Register >> 16) & 0xFF;
		strMessage[u8Index++] = 0; /* Starting Address */
		strMessage[u8Index++] = 0; /* Starting Address */
		strMessage[u8Index++] = 0; /* Number of data */
		strMessage[u8Index++] = u8DataLength;
		/* Number of data */
		if (pMsg->eRegAccess != 0)  // Write message
				{
			for ( u8i = 0; u8i < u8DataLength; u8i++ ) { /* Data */
				strMessage[u8Index++] = pMsg->ti64Data.u8[u8i]; /* Data */
			}
		}
		/* Compute CRC (Not functional) */
		/*
		 if (isUARTCrcEnabled != FALSE)
		 {
		 u16CRC = CRC16Check(pBuffer, u16Index);
		 strMessage[u16Index++] = (u16CRC >> 8) & 0x00FF;
		 strMessage[u16Index++] =  u16CRC & 0x00FF;
		 }*/

		strMessage[u8Index++] = 0;  // CRC (Not Enabled)
		strMessage[u8Index++] = 0;  // CRC (Not Enabled)

		strMessage[u8Index++] = SYNC_BYTE_BIN; /* End of frame */
		strMessage[u8Index++] = SYNC_BYTE_BIN;
		strMessage[u8Index++] = SYNC_BYTE_BIN;
		strMessage[u8Index++] = SYNC_BYTE_BIN;

		*pubLenght = u8Index;

	}
	else {

		if (pMsg->eRegAccess == READ) {
			sprintf(strMessage, "0x%02X %s 0x%lX\r", b8NodeID, "R", u32Register);
		}
		else {
			ti64Data.i64 = 0;
			for ( u8I = 0; u8I < u8DataLength; u8I++ ) {
				ti64Data.b8[u8I] = pMsg->ti64Data.b8[u8I];
			}

			switch (u8DataLength)
			{
			case 1:
				sprintf(strMessage, "0x%02X %s 0x%lX %d\r", b8NodeID, "W", u32Register,
						ti64Data.i8[0]);
				break;
			case 2:
				sprintf(strMessage, "0x%02X %s 0x%lX %d\r", b8NodeID, "W", u32Register,
						ti64Data.i16[0]);
				break;
			case 4:
				sprintf(strMessage, "0x%02X %s 0x%lX %ld\r", b8NodeID, "W", u32Register,
						ti64Data.i32[0]);
				break;
			case 5:
			case 6:
			case 7:
			case 8:

				sprintf(strMessage, "0x%02X %s 0x%lX %lld\r", b8NodeID, "W", u32Register,
						ti64Data.i64);
				break;

			default:
				return false;
			}
		}
		*pubLenght = (size_t) strlen(strMessage);
	}

	return true;
}

bool Ingenia_decodeMsg(const char* strMessage, TMessage *pMsg, bool isBinary)
{
	unsigned char b8NodeID;
	uint32_t u32Register;
	char* pb8Token;
	bool bSuccess;

	/* Verify parameters */
	if (pMsg == NULL || strMessage == NULL) {
		return false;
	}

	/* Start decoding message process */
	bSuccess = false;
	while (1) {
		if (isBinary) {
			uint16_t u16Index = 0, u16i;

			pMsg->b8ID = strMessage[u16Index++]; /* ModBus Address */
			u16Index += 2;
			pMsg->eRegAccess = (strMessage[u16Index++] & 0x01) == 0 ? READ : WRITE;
			u16Index++;
			pMsg->b8ID = strMessage[u16Index++]; /* Node ID */
			pMsg->u32Register = ((uint16_t) (strMessage[u16Index] << 8) & 0xFF00)
					| ((uint16_t) (strMessage[u16Index + 1]) & 0x00FF);
			u16Index += 2;
			pMsg->u32Register = (pMsg->u32Register) + ((uint32_t) strMessage[u16Index++] << 16);
			u16Index += 3;
			pMsg->b8DataLength = strMessage[u16Index++]; /* Number of data */
			if (pMsg->eRegAccess == WRITE) {
				pMsg->ti64Data.i64 = 0;
				for ( u16i = 0; u16i < pMsg->b8DataLength; u16i++ ) {
					pMsg->ti64Data.u8[u16i] = strMessage[u16Index++]; /* Data */
				}
			}
			bSuccess = true;
			break;
		}
		else {
			pb8Token = NULL;

			// Get the Node ID
			// Look for the first token " "
			if ((pb8Token = strtok((char*) strMessage, " ")) == NULL) {
				// Node not found in message
				break;
			}
			b8NodeID = (unsigned char) strtol(pb8Token, NULL, 0);
			if (b8NodeID > MAX_NUM_MC) {
				// Out of limits
				break;
			}
			pMsg->b8ID = b8NodeID;

			// Get the register
			if ((pb8Token = strtok(NULL, " ")) == NULL) {
				// Not found in message
				break;
			}

			// Get access type
			if (*pb8Token == 'R') {
				pMsg->eRegAccess = READ;
			}
			else if (*pb8Token == 'W') {
				pMsg->eRegAccess = WRITE;
			}
			else {
				// Unknown
				break;
			}

			// Get the register
			if ((pb8Token = strtok(NULL, " ")) == NULL) {
				// Not found in message
				break;
			}
			u32Register = (uint32_t) strtol(pb8Token, NULL, 0);
			pMsg->u32Register = (u32Register & 0xFFFFFF);

			// If the message is a reading the process stops here
			if (pMsg->eRegAccess == READ) {
				bSuccess = true;
				break;
			}

			// If the message is a writing, get the parameter
			if ((pb8Token = strtok(NULL, " ")) == NULL) {
				// Parameter not found in message
				break;
			}

			// Get data length
			pMsg->b8DataLength = MCL_MSG_MAXDATALENGTH;

			// Get data value
			if (strchr(pb8Token, 'x') != NULL) {   // Data is in hexadecimal base
				pMsg->ti64Data.u64 = strtoul(pb8Token, NULL, 16);
			}
			else {   // Data is in decimal base
				pMsg->ti64Data.u64 = strtoul(pb8Token, NULL, 10);
			}
			bSuccess = true;
			break;
		}
	}

	return bSuccess;
}

//
// Status Word Register (SWR) bits meaning
//
//  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// |  15      |     14       | 13         12 |      11      |   10    |   9       |      8       |   7     |     6     |    5   |    4    |   3   |    2      |    1     |      0     |
// | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// | Reserved |Init angle    | Operation     | Internal     | Target  | Remote    | Reseved      | Warning | Switch on | Quick  | Voltage | Fault | Operation | Switched |  Ready to  |
// |          |proc finished | mode specific | limit active | reached |           |              |         | disabled  | stop   | enabled |       | enabled   | on       | switch on  |
//  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
#define STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON     0x0001
#define STATUS_WORD_REGISTER_BITS_SWITCHED_ON            0x0002
#define STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED      0x0004
#define STATUS_WORD_REGISTER_BITS_FAULT                  0x0008
#define STATUS_WORD_REGISTER_BITS_VOLTAGE_ENABLED        0x0010
#define STATUS_WORD_REGISTER_BITS_QUICK_STOP             0x0020
#define STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED     0x0040
#define STATUS_WORD_REGISTER_BITS_WARNING                0x0080
#define STATUS_WORD_REGISTER_BITS_RESERVED               0x0100
#define STATUS_WORD_REGISTER_BITS_REMOTE                 0x0200
#define STATUS_WORD_REGISTER_BITS_TARGET_REACHED         0x0400
#define STATUS_WORD_REGISTER_BITS_INTERNAL_LIMIT_ACTIVE  0x0800
#define STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1      0x1000
#define STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2      0x2000
#define STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED    0x4000

//
// Control Word Register (CWR) bits meaning
//
//  --------------------------------------------------------------------------------
// | 15     9 |   8  |   7    | 6      4 |    3      |   2     |   1       |    0   |
// | -------------------------------------------------------------------------------
// | Reserved | Halt | Fault  | Mode     | Enable    | Quick   | Enable    | Switch |
// |          |      | reset  | specific | operation | stop    | voltage   | on     |
//  --------------------------------------------------------------------------------
//
#define CONTROL_WORD_REGISTER_BITS_SWITCH_ON             0x0001
#define CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE        0x0002
#define CONTROL_WORD_REGISTER_BITS_QUICK_STOP            0x0004
#define CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION      0x0008
#define CONTROL_WORD_REGISTER_BITS_MODE_SPECIFIC         0x0070
#define CONTROL_WORD_REGISTER_BITS_FAULT_RESET           0x0080
#define CONTROL_WORD_REGISTER_BITS_HALT                  0x0100
#define CONTROL_WORD_REGISTER_BITS_14_RESERVED           0x4000

// Useful object definitions (index, subindex)
#define OBJECT_CONTROL_WORD            0x6040,0x00
#define OBJECT_STATUS_WORD             0x6041,0x00

void Ingenia_sendStateMachineCommand(TServo *servo, const EStateMachineCommand eStateMachineCommand)
{
	bool isValidCommand = true;
	bool needsTransition = false;
	uint16_t u16TransitionAuxValue = 0;
	uint16_t u16ActualControlWord = 0;

	u16ActualControlWord = Ingenia_readReg(servo, OBJECT_CONTROL_WORD);
	switch (eStateMachineCommand)
	{
	case COMMAND_SHUTDOWN:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_SWITCH_ON
				| CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_SWITCH_ON:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION
				| CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON
				| CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE |
				CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_SWITCH_ON_AND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |=
				(CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE |
				CONTROL_WORD_REGISTER_BITS_QUICK_STOP | CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
		break;

	case COMMAND_DISABLE_VOLTAGE:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
				| CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		break;

	case COMMAND_QUICK_STOP:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_QUICK_STOP
				| CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE;
		break;

	case COMMAND_DISABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION
				| CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON
				| CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE |
				CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |=
				(CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE |
				CONTROL_WORD_REGISTER_BITS_QUICK_STOP | CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
		break;

	case COMMAND_FAULT_RESET:
		if (u16ActualControlWord & CONTROL_WORD_REGISTER_BITS_FAULT_RESET) {
			u16ActualControlWord &= ~CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
			needsTransition = true;
			u16TransitionAuxValue = CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		else {
			u16ActualControlWord |= CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		break;

	default:
		isValidCommand = false;
		break;
	}

	if (isValidCommand == true) {
		Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		if (needsTransition == true) {
			u16ActualControlWord |= u16TransitionAuxValue;
			Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		}
	}
}

void Ingenia_setStateMachineStatus(TServo *servo, const EStateMachineStatus eNewStateMachineStatus)
{
	EStateMachineStatus eCurrentStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &eCurrentStateMachineStatus);
	Ingenia_goToStatusFrom(servo, eCurrentStateMachineStatus, eNewStateMachineStatus);
}

void Ingenia_getStateMachineStatus(TServo *servo, EStateMachineStatus* peStateMachineStatus)
{
	uint16_t u16StatusWord;

	u16StatusWord = Ingenia_readReg(servo, OBJECT_STATUS_WORD);
	*peStateMachineStatus = Ingenia_decodeStatusWord(servo, u16StatusWord);
}

EStateMachineStatus Ingenia_decodeStatusWord(TServo *servo, uint16_t u16StatusWord)
{
	uint16_t u16LSBMask;
	EStateMachineStatus eStateMachineStatus;

	u16LSBMask = STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON
			| STATUS_WORD_REGISTER_BITS_SWITCHED_ON |
			STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED | STATUS_WORD_REGISTER_BITS_FAULT;

// ----------------------------------------------   //
//      Value (binary)  |           Status          //
// ----------------------------------------------   //
// xxxx xxxx x0xx 0000  | Not ready to switch on    //
// xxxx xxxx x1xx 0000  | Switched on disabled      //
// xxxx xxxx x01x 0001  | Ready to switch on        //
// xxxx xxxx x01x 0011  | Switched on               //
// xxxx xxxx x01x 0111  | Operation enabled         //
// xxxx xxxx x00x 0111  | Quick stop active         //
// xxxx xxxx x0xx 1111  | Fault reaction active     //
// xxxx xxxx x0xx 1000  | Fault                     //
// ----------------------------------------------   //
//

	servo->_isInitialAngleDeterminationProcessFinished = ((u16StatusWord
			& STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED)
			== STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED);

	eStateMachineStatus = STATUS_UNKNOWN;
	switch ((u16StatusWord & u16LSBMask))
	{
	case 0x00:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
		}
		else {
			eStateMachineStatus = STATUS_SWITCH_ON_DISABLED;
		}
		break;

	case 0x01:
		if ((u16StatusWord
				& (STATUS_WORD_REGISTER_BITS_QUICK_STOP
						| STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED)) == 0x20) {
			eStateMachineStatus = STATUS_READY_TO_SWITCH_ON;
		}
		break;

	case 0x03:
		if ((u16StatusWord
				& (STATUS_WORD_REGISTER_BITS_QUICK_STOP
						| STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED)) == 0x20) {
			eStateMachineStatus = STATUS_SWITCH_ON;
		}
		break;

	case 0x07:
		if ((u16StatusWord
				& (STATUS_WORD_REGISTER_BITS_QUICK_STOP
						| STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED)) == 0x20) {
			eStateMachineStatus = STATUS_OPERATION_ENABLED;
		}
		else if ((u16StatusWord
				& (STATUS_WORD_REGISTER_BITS_QUICK_STOP
						| STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED)) == 0x00) {
			eStateMachineStatus = STATUS_QUICK_STOP_ACTIVE;
		}
		break;

	case 0x0F:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT_REACTION_ACTIVE;
		}
		break;

	case 0x08:
		if ((u16StatusWord & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) == 0x00) {
			eStateMachineStatus = STATUS_FAULT;
		}
		break;

	default:
		eStateMachineStatus = STATUS_UNKNOWN;
		break;
	}

	return eStateMachineStatus;
}

void Ingenia_gotoStatus(TServo *servo, const EStateMachineStatus eDestinationStateMachineStatus)
{
	EStateMachineStatus tCurrentStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
	Ingenia_getStateMachineStatus(servo, &tCurrentStateMachineStatus);

	while (tCurrentStateMachineStatus != eDestinationStateMachineStatus) {
		EStateMachineStatus nextState = eDestinationStateMachineStatus;
		switch (tCurrentStateMachineStatus)
		{
		case STATUS_SWITCH_ON_DISABLED:
			if ((eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else {
				return;
			}
			break;
		case STATUS_READY_TO_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON);
				nextState = STATUS_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON_AND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_OPERATION_ENABLED:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_OPERATION);
				nextState = STATUS_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_QUICK_STOP);
				nextState = STATUS_QUICK_STOP_ACTIVE;
			}
			else {
				return;
			}
			break;
		case STATUS_QUICK_STOP_ACTIVE:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_FAULT:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_FAULT_RESET);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		default:
		{
			return;
		}
			break;
		}

		Ingenia_verifyStatusIsReached(servo, nextState);
		tCurrentStateMachineStatus = nextState;
	}
}

void Ingenia_goToStatusFrom(TServo *servo, const EStateMachineStatus eCurrentStateMachineStatus,
		const EStateMachineStatus eDestinationStateMachineStatus)
{
	EStateMachineStatus tCurrentStateMachineStatus = eCurrentStateMachineStatus;
	while (tCurrentStateMachineStatus != eDestinationStateMachineStatus) {
		EStateMachineStatus nextState = eDestinationStateMachineStatus;
		switch (tCurrentStateMachineStatus)
		{
		case STATUS_SWITCH_ON_DISABLED:
			if ((eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else {
				return;
			}
			break;
		case STATUS_READY_TO_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON);
				nextState = STATUS_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SWITCH_ON_AND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_SWITCH_ON:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if ((eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_ENABLE_OPERATION);
				nextState = STATUS_OPERATION_ENABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_OPERATION_ENABLED:
			if (eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else if (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_SHUTDOWN);
				nextState = STATUS_READY_TO_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_SWITCH_ON) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_OPERATION);
				nextState = STATUS_SWITCH_ON;
			}
			else if (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_QUICK_STOP);
				nextState = STATUS_QUICK_STOP_ACTIVE;
			}
			else {
				return;
			}
			break;
		case STATUS_QUICK_STOP_ACTIVE:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_DISABLE_VOLTAGE);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		case STATUS_FAULT:
			if ((eDestinationStateMachineStatus == STATUS_SWITCH_ON_DISABLED)
					|| (eDestinationStateMachineStatus == STATUS_READY_TO_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_SWITCH_ON)
					|| (eDestinationStateMachineStatus == STATUS_OPERATION_ENABLED)
					|| (eDestinationStateMachineStatus == STATUS_QUICK_STOP_ACTIVE)) {
				Ingenia_sendStateMachineCommand(servo, COMMAND_FAULT_RESET);
				nextState = STATUS_SWITCH_ON_DISABLED;
			}
			else {
				return;
			}
			break;
		default:
		{
			return;
		}
			break;
		}

		Ingenia_verifyStatusIsReached(servo, nextState);
		tCurrentStateMachineStatus = nextState;
	}
}

bool Ingenia_verifyStatusIsReached(TServo *servo, const EStateMachineStatus eNextStateMachineStatus)
{
	EStateMachineStatus eActualStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);

	uint32_t started_waiting_at = HAL_GetTick();
	uint32_t millis = 0;
	while (eNextStateMachineStatus != eActualStateMachineStatus) {
		millis = HAL_GetTick();
		// Check if timeout excedeed
		if (millis - started_waiting_at > 1000)
			return false;
		Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);
	}
	return true;

}

void Ingenia_enableMotor(TServo *servo)
{
//	Ingenia_setStateMachineStatus(servo, STATUS_OPERATION_ENABLED);
	Ingenia_gotoStatus(servo, STATUS_OPERATION_ENABLED);
}

void Ingenia_disableMotor(TServo * servo)
{
	EStateMachineStatus currentStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &currentStateMachineStatus);

	if (currentStateMachineStatus == STATUS_OPERATION_ENABLED) {
		Ingenia_setStateMachineStatus(servo, STATUS_SWITCH_ON);
	}
}

#define TARGET_POSITION_INDEX       0x607A
#define TARGET_POSITION_SUBINDEX    0x0
#define VELOCITY_INDEX              0x60FF
#define VELOCITY_SUBINDEX           0x0
#define TORQUE_INDEX                0x6071
#define TORQUE_SUBINDEX             0x0

void Ingenia_setTargetVelocity(TServo * servo, int32_t value)
{
	Ingenia_write_i32(servo, VELOCITY_INDEX, VELOCITY_SUBINDEX, value);
}

void Ingenia_setTargetTorque(TServo * servo, int16_t value)
{
	Ingenia_write_i16(servo, TORQUE_INDEX, TORQUE_SUBINDEX, value);
}

#define MODES_OF_OPERATION_INDEX				0x6060
#define MODES_OF_OPERATION_SUBINDEX				0x00
#define MODE_SPECIFIC_BITS_NEW_SETPOINT   		0x0010
#define MODE_SPECIFIC_BITS_CHANGE_SET			0x0020
#define MODE_SPECIFIC_BITS_ABS_REL				0x0040

void Ingenia_setModeOfOperation(TServo * servo, EDriveModes driverMode)
{
	Ingenia_write_u8(servo, MODES_OF_OPERATION_INDEX, MODES_OF_OPERATION_SUBINDEX,
			(uint8_t) driverMode);
}

void Ingenia_setTargetPositionAdv(TServo * servo, int32_t value, bool isImmediate, bool isRelative,
bool isHaltEnabled)
{
	Ingenia_write_i32(servo, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, value);

	uint16_t u16ActualControlWord = 0;
	u16ActualControlWord = Ingenia_readReg(servo, OBJECT_CONTROL_WORD);

	if ((u16ActualControlWord & MODE_SPECIFIC_BITS_NEW_SETPOINT) > 0) {  // Low new setpoint flag if needed
		u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
		Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
	}

	/* Raise New Setpoint Bit */
	u16ActualControlWord |= 1 << 4;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_CHANGE_SET);
	u16ActualControlWord |= isImmediate << 5;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_ABS_REL);
	u16ActualControlWord |= isRelative << 6;

	u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_HALT);
	u16ActualControlWord |= isHaltEnabled << 8;

	Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);

	/* Low New Setpoint */
	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
	Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
}

#define POSITION_ACTUAL_INDEX         0x6064
#define POSITION_ACTUAL_SUBINDEX      0x0
#define TORQUE_ACTUAL_INDEX           0x6077
#define TORQUE_ACTUAL_SUBINDEX        0x0
#define VELOCITY_ACTUAL_INDEX         0x606C
#define VELOCITY_ACTUAL_SUBINDEX      0x0

void Ingenia_setTargetPosition(TServo * servo, int32_t value)
{
	Ingenia_setTargetPositionAdv(servo, value, true, false, false);
}

int32_t Ingenia_getActualPosition(TServo * servo)
{
	return Ingenia_readReg(servo, POSITION_ACTUAL_INDEX, POSITION_ACTUAL_SUBINDEX);
}

int32_t Ingenia_getActualVelocity(TServo * servo)
{
	return Ingenia_readReg(servo, VELOCITY_ACTUAL_INDEX, VELOCITY_ACTUAL_SUBINDEX);
}

int16_t Ingenia_getActualTorque(TServo * servo)
{
	return Ingenia_readReg(servo, TORQUE_ACTUAL_INDEX, TORQUE_ACTUAL_SUBINDEX);
}

void Ingenia_enableBinary(TServo * servo)
{
	uint8_t _node = servo->_u8Node;
	servo->_u8Node = 0;
	Ingenia_write_u8(servo, 0x2000, 0x8, 1);
	servo->_u8Node = _node;
	servo->_isBinaryEnabled = true;
}

void Ingenia_disableBinary(TServo * servo)
{
	uint8_t _node = servo->_u8Node;
	servo->_u8Node = 0;
	Ingenia_write_u8(servo, 0x2000, 0x8, 0);
	servo->_u8Node = _node;
	servo->_isBinaryEnabled = false;
}

#define STATUSWORD_INDEX         0x6041
#define STATUSWORD_SUBINDEX      0x0

uint16_t Ingenia_getStatusword(TServo * servo)
{
	return Ingenia_readReg(servo, STATUSWORD_INDEX, STATUSWORD_SUBINDEX);
}

bool Ingenia_statuswordIsReadyToSwitchOn(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON) > 0;
}

bool Ingenia_statuswordIsSwitchedOn(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCHED_ON) > 0;
}

bool Ingenia_statuswordIsOperationEnabled(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED) > 0;
}

bool Ingenia_statuswordIsFault(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_FAULT) > 0;
}

bool Ingenia_statuswordIsVoltageEnabled(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_VOLTAGE_ENABLED) > 0;
}

bool Ingenia_statuswordIsQuickStop(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_QUICK_STOP) > 0;
}

bool Ingenia_statuswordIsSwitchOnDisabled(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) > 0;
}

bool Ingenia_statuswordIsWarning(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_WARNING) > 0;
}

bool Ingenia_statuswordIsRemote(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_REMOTE) > 0;
}

bool Ingenia_statuswordIsTargetReached(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0;
}

bool Ingenia_statuswordIsInternalLimitActive(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_INTERNAL_LIMIT_ACTIVE) > 0;
}

bool Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED) > 0;
}

bool Ingenia_homingStatusIsInProgress(TServo * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}

bool Ingenia_homingStatusIsError(TServo * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) > 0;
}

bool Ingenia_homingStatusIsSuccess(TServo * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}

bool Ingenia_homingStatusIsAttained(TServo * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}

bool Ingenia_homingStatusIsInterrupted(TServo * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}

bool Ingenia_homingStatusNotStarted(TServo * servo)
{
	return Ingenia_homingStatusIsInterrupted(servo);
}

void Ingenia_doHoming(TServo * servo, int8_t i8HomingMethod)
{
	uint16_t controlWord;

	Ingenia_setModeOfOperation(servo, DRIVE_MODE_HOMING);
	Ingenia_write_i8(servo, 0x6098, 0x0, i8HomingMethod);

	Ingenia_enableMotor(servo);

	controlWord = Ingenia_readReg(servo, OBJECT_CONTROL_WORD);
	controlWord &= ~0x0010;
	Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, controlWord);
	controlWord = Ingenia_readReg(servo, OBJECT_CONTROL_WORD);
	controlWord |= 0x0010;
	Ingenia_write_u16(servo, OBJECT_CONTROL_WORD, controlWord);
}
