/*
 * Ingenia_CanServoDriver.c
 *
 *  Created on: Feb 14, 2020
 *      Author: miftakur
 */

#include "Ingenia_CanServoDriver.h"
#include <string.h>

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

#define SDO_DOWNLOAD_REQUEST_BITS	0x20
#define SDO_DOWNLOAD_RESPONSE_BITS	0x60
#define SDO_UPLOAD_REQUEST_BITS		0x40
#define SDO_UPLOAD_RESPONSE_BITS	0x40

#define SDO_DATA_SIZE_MASK			0x0C
#define SDO_DATA_SIZE_1				0x0C
#define SDO_DATA_SIZE_2				0x08
#define SDO_DATA_SIZE_3				0x04
#define SDO_DATA_SIZE_4				0x00

#define SDO_E_TRANSFER_BIT			0x02
#define SDO_S_SIZE_INDICATOR_BIT	0x01

typedef struct
{
	uint8_t _id;
	Servo_t *servo;
} SlaveNodeId_t;

typedef struct
{
	SlaveNodeId_t _node[127];
	uint8_t size;
} SlaveNode_t;
SlaveNode_t slaveNodeBank;

CAN_TxHeaderTypeDef canTxHeader;
uint32_t canTxMailBox;
CAN_HandleTypeDef *Ingenia_hcan;

HAL_StatusTypeDef Ingenia_begin(CAN_HandleTypeDef *hcan)
{
	slaveNodeBank.size = 0;
	for ( uint8_t i = 0; i < 127; i++ ) {
		slaveNodeBank._node[i]._id = 0;
		slaveNodeBank._node[i].servo = NULL;
	}

	Ingenia_hcan = hcan;

	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x80 << 5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x880 << 5;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(Ingenia_hcan, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x700 << 5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x780 << 5;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(Ingenia_hcan, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}
	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(Ingenia_hcan) != HAL_OK) {
		/* Start Error */
		return HAL_ERROR;
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(Ingenia_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		return HAL_ERROR;
	}

	/*##-5- Configure Transmission process #####################################*/
	canTxHeader.StdId = 0x1;
	canTxHeader.ExtId = 0x01;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = CAN_DATA_MAX;
	canTxHeader.TransmitGlobalTime = DISABLE;

	return HAL_OK;
}

static HAL_StatusTypeDef Ingenia_slave_add(Servo_t *servo)
{
	uint8_t currentBankSize = slaveNodeBank.size;

	if ((servo->_u8Node < 1) & (servo->_u8Node > 127))
		return HAL_ERROR;

	/* check if the node already exist in the banks */
	for ( int i = 0; i < currentBankSize; i++ ) {
		if (slaveNodeBank._node[i]._id == servo->_u8Node) {
			return HAL_ERROR;
		}
	}

	slaveNodeBank._node[currentBankSize]._id = servo->_u8Node;
	slaveNodeBank._node[currentBankSize].servo = servo;
	slaveNodeBank._node[currentBankSize].servo->_isRegistered = 1;
	slaveNodeBank.size++;

	return HAL_OK;
}

HAL_StatusTypeDef Ingenia_init(Servo_t *servo, uint8_t node)
{
	can_buffer_flush(&servo->buffer.bufEMER);
	can_buffer_flush(&servo->buffer.bufNMT);
	can_buffer_flush(&servo->buffer.bufTPDO);
	can_buffer_flush(&servo->buffer.bufTSDO);
	servo->_u8Node = node;
	servo->_isRegistered = 0;
	servo->_isInitialAngleDeterminationProcessFinished = 0;

	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);

	return Ingenia_slave_add(servo);
}

HAL_StatusTypeDef Ingenia_node_remove(Servo_t *servo)
{
	if (slaveNodeBank.size == 0)
		return HAL_ERROR;

	/* check if the node already exist in the banks */
	for ( int i = 0; i < slaveNodeBank.size; i++ ) {
		if (slaveNodeBank._node[i]._id == servo->_u8Node) {
			servo->_u8Node = 0;
			servo->_isRegistered = 0;
			if (i < slaveNodeBank.size - 1) {
				for ( int j = i; j < slaveNodeBank.size - 1; j++ ) {
					slaveNodeBank._node[j]._id = slaveNodeBank._node[j + 1]._id;
					slaveNodeBank._node[j].servo = slaveNodeBank._node[j + 1].servo;
				}
			}
			slaveNodeBank._node[slaveNodeBank.size - 1]._id = 0;
			slaveNodeBank._node[slaveNodeBank.size - 1].servo = NULL;
			slaveNodeBank.size--;

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

/**
 * @brief  Rx FIFO 0 message pending callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hcan);

	Ingenia_IRQHandler();
}

HAL_StatusTypeDef Ingenia_IRQHandler()
{
	CAN_Data_t data;
	uint32_t id;
	uint32_t idType = 0;

	if (HAL_CAN_GetRxMessage(Ingenia_hcan, CAN_RX_FIFO0, &data.canRxHeader, data.rxData)
			== HAL_OK) {
		if (data.canRxHeader.IDE == CAN_ID_STD) {
			id = data.canRxHeader.StdId;

			if (id < COB_NMT_CTRL)
				id %= 0x80;
			else if (id < COB_NMT_CTRL + 0x80)
				id -= COB_NMT_CTRL;

			idType = data.canRxHeader.StdId - id;

			/* check if the node already exist in the banks */
			for ( int i = 0; i < slaveNodeBank.size; i++ ) {
				if (slaveNodeBank._node[i]._id == id) {
					if (idType == COB_EMERGENCY) {
						can_buffer_write(&slaveNodeBank._node[i].servo->buffer.bufEMER, &data);
						Ingenia_emer_callback(&slaveNodeBank._node[i].servo->buffer.bufEMER);
					}
					else if (idType == COB_NMT_CTRL) {
						can_buffer_write(&slaveNodeBank._node[i].servo->buffer.bufNMT, &data);
						Ingenia_nmt_callback(&slaveNodeBank._node[i].servo->buffer.bufNMT);
					}
					else if (idType == COB_TSDO) {
						can_buffer_write(&slaveNodeBank._node[i].servo->buffer.bufTSDO, &data);
						Ingenia_tsdo_callback(&slaveNodeBank._node[i].servo->buffer.bufTSDO);
					}
					else {
						can_buffer_write(&slaveNodeBank._node[i].servo->buffer.bufTPDO, &data);
						Ingenia_tpdo_callback(&slaveNodeBank._node[i].servo->buffer.bufTPDO);
					}
				}
			}

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

__weak void Ingenia_emer_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_nmt_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

__weak void Ingenia_tsdo_callback(CAN_Buffer_t *buffer)
{
	UNUSED(buffer);
}

HAL_StatusTypeDef Ingenia_buffer_available(CAN_Buffer_t *buf, CAN_Data_t *data)
{
	if (can_buffer_available(buf)) {
		can_buffer_read(buf, data);
		return HAL_OK;
	}
	return HAL_ERROR;
}

void Ingenia_write_nmt(Servo_t *servo, NmtModes_e mode)
{
	uint8_t canTxBuffer[CAN_DATA_MAX];

	canTxHeader.StdId = COB_NMT_SERVICE;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 2;
	canTxHeader.RTR = CAN_RTR_DATA;
	memset(canTxBuffer, 0, CAN_DATA_MAX);
	canTxBuffer[0] = mode;
	canTxBuffer[1] = servo->_u8Node;

	/* Start the Transmission process */
	HAL_CAN_AddTxMessage(Ingenia_hcan, &canTxHeader, canTxBuffer, &canTxMailBox);
}

uint32_t Ingenia_read_sdo(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex, uint8_t *bIsValid)
{
	/* read register using SDO */
	uint8_t canTxBuffer[8];
	uint8_t isValid = 0;
	uint32_t millis = 0;
	uint32_t elapseTime;
	uint32_t sendTimer = 0;
	CAN_Data_t cData;
	uint16_t u16IndexFeed;
	uint8_t u8SubIndexFeed;
	uint32_t u32Result = 0;

	/* flush current buffer */
	can_buffer_flush(&servo->buffer.bufTSDO);
	memset(canTxBuffer, 0, 8);

	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = 8;

	canTxHeader.StdId = COB_RSDO + servo->_u8Node;
	canTxBuffer[0] = 0x40;  //css[5..7]=2(upload), bit[0..4]=x
	canTxBuffer[1] = u16Index & 0xFF;
	canTxBuffer[2] = u16Index >> 8 & 0xFF;
	canTxBuffer[3] = u8SubIndex;

	elapseTime = HAL_GetTick() + 1000;
	while (!isValid) {
		millis = HAL_GetTick();
		/* check if timeout exceeded */
		if (millis >= elapseTime)
			break;

		if (millis >= sendTimer) {
			/* Start the Transmission process */
			HAL_CAN_AddTxMessage(Ingenia_hcan, &canTxHeader, canTxBuffer, &canTxMailBox);
			sendTimer = millis + 200;
		}

		if (can_buffer_available(&servo->buffer.bufTSDO)) {
			if (can_buffer_read(&servo->buffer.bufTSDO, &cData) > 0) {
				/* check id sender (must be from CBO_TSDO & match index & sub-index */
				u16IndexFeed = (uint16_t) cData.rxData[2] << 8 | cData.rxData[1];
				u8SubIndexFeed = cData.rxData[3];

				if ((u16IndexFeed == u16Index) && (u8SubIndexFeed == u8SubIndex)) {
					u32Result = (uint32_t) cData.rxData[7] << 24 | (uint32_t) cData.rxData[6] << 16
							| (uint32_t) cData.rxData[5] << 8 | (uint32_t) cData.rxData[4];

					isValid = 1;
				}
			}
		}
	}

	*bIsValid = isValid;

	return u32Result;
}

uint32_t Ingenia_read_reg_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex)
{
	uint32_t u32Result = 0;
	uint8_t isValid = 0;

	while (!isValid)
		u32Result = Ingenia_read_sdo(servo, u16Index, u8SubIndex, &isValid);

	return u32Result;
}

static HAL_StatusTypeDef Ingenia_write_sdo(Servo_t *servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint8_t objSize, uint32_t value)
{
	uint8_t canTxBuffer[8];
	uint8_t sdoDataSize = 0;
	uint32_t elapseTime;
	uint8_t isValid = 0;
	CAN_Data_t dummy;

	memset(canTxBuffer, 0, CAN_DATA_MAX);
	switch (objSize)
	{
	case 1:
		sdoDataSize = SDO_DATA_SIZE_1;
		canTxBuffer[4] = value;
		break;
	case 2:
		sdoDataSize = SDO_DATA_SIZE_2;
		canTxBuffer[4] = value & 0xFF;
		canTxBuffer[5] = value >> 8 & 0xFF;
		break;
	case 4:
		sdoDataSize = SDO_DATA_SIZE_4;
		canTxBuffer[4] = value & 0xFF;
		canTxBuffer[5] = value >> 8 & 0xFF;
		canTxBuffer[6] = value >> 16 & 0xFF;
		canTxBuffer[7] = value >> 24 & 0xFF;
		break;
	}

	canTxBuffer[0] = SDO_DOWNLOAD_REQUEST_BITS | SDO_E_TRANSFER_BIT | SDO_S_SIZE_INDICATOR_BIT
			| sdoDataSize;

	canTxBuffer[1] = u16Index & 0xFF;
	canTxBuffer[2] = (u16Index >> 8) & 0xFF;
	canTxBuffer[3] = u8SubIndex;

	canTxHeader.StdId = COB_RSDO | servo->_u8Node;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	canTxHeader.RTR = CAN_RTR_DATA;

	can_buffer_flush(&servo->buffer.bufTSDO);
	/* Start the Transmission process */
	if (HAL_CAN_AddTxMessage(Ingenia_hcan, &canTxHeader, canTxBuffer, &canTxMailBox) == HAL_OK) {
		/* ignore reply */
		elapseTime = HAL_GetTick() + 1000;
		isValid = 0;
		while (!isValid) {
			if (HAL_GetTick() >= elapseTime)
				break;
			if (can_buffer_available(&servo->buffer.bufTSDO)) {
				can_buffer_read(&servo->buffer.bufTSDO, &dummy);
				if (((dummy.rxData[0] & SDO_DOWNLOAD_RESPONSE_BITS) == SDO_DOWNLOAD_RESPONSE_BITS)
						&& (dummy.rxData[1] == (u16Index & 0xFF))
						&& (dummy.rxData[2] == ((u16Index >> 8) & 0xFF))
						&& (dummy.rxData[3] == u8SubIndex)) {

					isValid = 1;
					break;
				}
			}
		}
		if (isValid)
			return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef Ingenia_write_sdo_u32(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint32_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 4, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i32(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int32_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 4, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_u16(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint16_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 2, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i16(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int16_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 2, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_u8(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		uint8_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 1, value);
}

HAL_StatusTypeDef Ingenia_write_sdo_i8(Servo_t * servo, uint16_t u16Index, uint8_t u8SubIndex,
		int8_t value)
{
	return Ingenia_write_sdo(servo, u16Index, u8SubIndex, 1, value);
}

static StateMachineStatus_e Ingenia_decodeStatusWord(Servo_t *servo, uint16_t u16StatusWord)
{
	uint16_t u16LSBMask;
	StateMachineStatus_e eStateMachineStatus = STATUS_UNKNOWN;

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

static void Ingenia_getStateMachineStatus(Servo_t *servo,
		StateMachineStatus_e* peStateMachineStatus)
{
	uint16_t u16StatusWord = Ingenia_read_reg_sdo(servo, OBJECT_STATUS_WORD);
	*peStateMachineStatus = Ingenia_decodeStatusWord(servo, u16StatusWord);
}

static void Ingenia_sendStateMachineCommand(Servo_t *servo,
		const StateMachineCommand_e eStateMachineCommand)
{
	uint8_t isValidCommand = 1;
	uint8_t needsTransition = 0;
	uint16_t u16TransitionAuxValue = 0;
	uint16_t u16ActualControlWord = 0;

	u16ActualControlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
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
		u16ActualControlWord |=
				(CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
						| CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_SWITCH_ON_AND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON
				| CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE | CONTROL_WORD_REGISTER_BITS_QUICK_STOP
				| CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
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
		u16ActualControlWord |=
				(CONTROL_WORD_REGISTER_BITS_SWITCH_ON | CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE
						| CONTROL_WORD_REGISTER_BITS_QUICK_STOP);
		break;

	case COMMAND_ENABLE_OPERATION:
		u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_FAULT_RESET);
		u16ActualControlWord |= (CONTROL_WORD_REGISTER_BITS_SWITCH_ON
				| CONTROL_WORD_REGISTER_BITS_ENABLE_VOLTAGE | CONTROL_WORD_REGISTER_BITS_QUICK_STOP
				| CONTROL_WORD_REGISTER_BITS_ENABLE_OPERATION);
		break;

	case COMMAND_FAULT_RESET:
		if (u16ActualControlWord & CONTROL_WORD_REGISTER_BITS_FAULT_RESET) {
			u16ActualControlWord &= ~CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
			needsTransition = 1;
			u16TransitionAuxValue = CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		else {
			u16ActualControlWord |= CONTROL_WORD_REGISTER_BITS_FAULT_RESET;
		}
		break;

	default:
		isValidCommand = 0;
		break;
	}

	if (isValidCommand == 1) {
		Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		if (needsTransition == 1) {
			u16ActualControlWord |= u16TransitionAuxValue;
			Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
		}
	}
}

static uint8_t Ingenia_verifyStatusIsReached(Servo_t *servo,
		const StateMachineStatus_e eNextStateMachineStatus)
{
	StateMachineStatus_e eActualStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);

	uint32_t started_waiting_at = HAL_GetTick();
	uint32_t millis = 0;
	while (eNextStateMachineStatus != eActualStateMachineStatus) {
		millis = HAL_GetTick();
		// Check if timeout excedeed
		if (millis - started_waiting_at > 1000)
			return 0;
		Ingenia_getStateMachineStatus(servo, &eActualStateMachineStatus);
	}
	return 1;

}

void Ingenia_gotoStatus(Servo_t *servo, const StateMachineStatus_e eDestinationStateMachineStatus)
{
	StateMachineStatus_e tCurrentStateMachineStatus = STATUS_NOT_READY_TO_SWITCH_ON;
	Ingenia_getStateMachineStatus(servo, &tCurrentStateMachineStatus);

	while (tCurrentStateMachineStatus != eDestinationStateMachineStatus) {
		StateMachineStatus_e nextState = eDestinationStateMachineStatus;
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

// Motor enable/disable functions
void Ingenia_enableMotor(Servo_t * servo)
{
	Ingenia_gotoStatus(servo, STATUS_OPERATION_ENABLED);
}

void Ingenia_disableMotor(Servo_t * servo)
{
	StateMachineStatus_e currentStateMachineStatus;
	Ingenia_getStateMachineStatus(servo, &currentStateMachineStatus);

	if (currentStateMachineStatus == STATUS_OPERATION_ENABLED) {
		Ingenia_gotoStatus(servo, STATUS_SWITCH_ON);
	}

}

// Homing
void Ingenia_doHoming(Servo_t * servo, int8_t i8HomingMethod)
{
	uint16_t controlWord = 0;

	Ingenia_setModeOfOperation(servo, DRIVE_MODE_HOMING);
	Ingenia_write_sdo_i8(servo, 0x6098, 0, i8HomingMethod);

	Ingenia_enableMotor(servo);

	controlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
	controlWord &= ~0x0010;
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, controlWord);
	controlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);
	controlWord |= 0x0010;
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, controlWord);
}

#define MODES_OF_OPERATION_INDEX				0x6060
#define MODES_OF_OPERATION_SUBINDEX				0x00
#define MODE_SPECIFIC_BITS_NEW_SETPOINT   		0x0010
#define MODE_SPECIFIC_BITS_CHANGE_SET			0x0020
#define MODE_SPECIFIC_BITS_ABS_REL				0x0040

// Motion funcions
void Ingenia_setModeOfOperation(Servo_t * servo, DriveModes_e driverMode)
{
	Ingenia_write_sdo_u8(servo, MODES_OF_OPERATION_INDEX, MODES_OF_OPERATION_SUBINDEX,
			(uint8_t) driverMode);
}

#define TARGET_POSITION_INDEX       0x607A
#define TARGET_POSITION_SUBINDEX    0x0
#define VELOCITY_INDEX              0x60FF
#define VELOCITY_SUBINDEX           0x0
#define TORQUE_INDEX                0x6071
#define TORQUE_SUBINDEX             0x0

void Ingenia_setTargetVelocity(Servo_t * servo, int32_t value)
{
	Ingenia_write_sdo_i32(servo, VELOCITY_INDEX, VELOCITY_SUBINDEX, value);
}

void Ingenia_setTargetTorque(Servo_t * servo, int16_t value)
{
	Ingenia_write_sdo_i16(servo, TORQUE_INDEX, TORQUE_SUBINDEX, value);
}

void Ingenia_setTargetPosition(Servo_t * servo, int32_t value)
{
	Ingenia_setTargetPositionAdv(servo, value, 1, 0, 0);
}

void Ingenia_setTargetPositionAdv(Servo_t * servo, int32_t value, uint8_t isImmediate,
		uint8_t isRelative, uint8_t isHaltEnabled)
{
	uint16_t u16ActualControlWord = 0;

	Ingenia_write_sdo_i32(servo, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, value);

	u16ActualControlWord = Ingenia_read_reg_sdo(servo, OBJECT_CONTROL_WORD);

	/* Low new setpoint flag if needed */
	if ((u16ActualControlWord & MODE_SPECIFIC_BITS_NEW_SETPOINT) > 0) {
		u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
		Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
	}

	/* Raise New Setpoint Bit */
	u16ActualControlWord |= 1 << 4;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_CHANGE_SET);
	u16ActualControlWord |= (uint16_t) isImmediate << 5;

	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_ABS_REL);
	u16ActualControlWord |= (uint16_t) isRelative << 6;

	u16ActualControlWord &= ~(CONTROL_WORD_REGISTER_BITS_HALT);
	u16ActualControlWord |= (uint16_t) isHaltEnabled << 8;

	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);

	/* Low New Setpoint */
	u16ActualControlWord &= ~(MODE_SPECIFIC_BITS_NEW_SETPOINT);
	Ingenia_write_sdo_u16(servo, OBJECT_CONTROL_WORD, u16ActualControlWord);
}

#define POSITION_ACTUAL_INDEX         0x6064
#define POSITION_ACTUAL_SUBINDEX      0x0
#define TORQUE_ACTUAL_INDEX           0x6077
#define TORQUE_ACTUAL_SUBINDEX        0x0
#define VELOCITY_ACTUAL_INDEX         0x606C
#define VELOCITY_ACTUAL_SUBINDEX      0x0

int32_t Ingenia_getActualPosition(Servo_t * servo)
{
	return Ingenia_read_reg_sdo(servo, POSITION_ACTUAL_INDEX, POSITION_ACTUAL_SUBINDEX);
}

int32_t Ingenia_getActualVelocity(Servo_t * servo)
{
	return Ingenia_read_reg_sdo(servo, VELOCITY_ACTUAL_INDEX, VELOCITY_ACTUAL_SUBINDEX);
}
int16_t Ingenia_getActualTorque(Servo_t * servo)
{
	return Ingenia_read_reg_sdo(servo, TORQUE_ACTUAL_INDEX, TORQUE_ACTUAL_SUBINDEX);
}

// Statusword functions
uint16_t Ingenia_getStatusword(Servo_t * servo)
{
	return Ingenia_read_reg_sdo(servo, OBJECT_STATUS_WORD);
}
uint8_t Ingenia_statuswordIsReadyToSwitchOn(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_READY_TO_SWITCH_ON) > 0;
}
uint8_t Ingenia_statuswordIsSwitchedOn(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCHED_ON) > 0;
}
uint8_t Ingenia_statuswordIsOperationEnabled(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPERATION_ENABLED) > 0;
}
uint8_t Ingenia_statuswordIsFault(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_FAULT) > 0;
}
uint8_t Ingenia_statuswordIsVoltageEnabled(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_VOLTAGE_ENABLED) > 0;
}
uint8_t Ingenia_statuswordIsQuickStop(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_QUICK_STOP) > 0;
}
uint8_t Ingenia_statuswordIsSwitchOnDisabled(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_SWITCH_ON_DISABLED) > 0;
}
uint8_t Ingenia_statuswordIsWarning(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_WARNING) > 0;
}
uint8_t Ingenia_statuswordIsRemote(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_REMOTE) > 0;
}
uint8_t Ingenia_statuswordIsTargetReached(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0;
}
uint8_t Ingenia_statuswordIsInternalLimitActive(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_INTERNAL_LIMIT_ACTIVE) > 0;
}
uint8_t Ingenia_statuswordIsInitialAngleDeterminationProcessFinished(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_ANGLE_PROC_FINISHED) > 0;
}

// Homing status functions
uint8_t Ingenia_homingStatusIsInProgress(Servo_t * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsError(Servo_t * servo)
{
	return (Ingenia_getStatusword(servo) & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) > 0;
}
uint8_t Ingenia_homingStatusIsSuccess(Servo_t * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsAttained(Servo_t * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusIsInterrupted(Servo_t * servo)
{
	uint16_t _statusWord = Ingenia_getStatusword(servo);

	return ((_statusWord & STATUS_WORD_REGISTER_BITS_TARGET_REACHED) > 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_1) == 0)
			&& ((_statusWord & STATUS_WORD_REGISTER_BITS_OPMODE_SPECIFIC_2) == 0);
}
uint8_t Ingenia_homingStatusNotStarted(Servo_t * servo)
{
	return Ingenia_homingStatusIsInterrupted(servo);
}
