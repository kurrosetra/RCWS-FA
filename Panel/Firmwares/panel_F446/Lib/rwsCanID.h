/*
 * rwsCanID.h
 *
 *  Created on: Jan 10, 2019
 *      Author: miftakur
 */
#ifndef RWSCANID_H_
#define RWSCANID_H_

#define CAN_ID_RWS_MOTOR			0x300
#define CAN_ID_RWS_MTR_STAB_ANGLE	0x301
#define CAN_ID_RWS_MTR_STAB_SPD		0x302
#define CAN_ID_RWS_PNL_MTR			0x310
#define CAN_ID_RWS_PNL_STAB_MODE	0x311
#define CAN_ID_RWS_BUTTON			0x320
#define CAN_ID_RWS_OPT_LRF			0x330
#define CAN_ID_RWS_OPT_CAM			0x332
#define CAN_ID_RWS_IMU				0x340
#define CAN_ID_RWS_IMU_YPR			0x341

#define C_AZ_FULLSWING				597200UL
#define C_TO_DEG_AZ(x)				((float)x * 0.000602813127930f)
#define C_TO_DEG_EL(x)				((float)x * 0.000795622951065f)
#define DEG_TO_C_AZ(x)				(x * 1658.888888888889f)
#define DEG_TO_C_EL(x)				(x * 1306.021505376344f)

#endif /* RWSCANID_H_ */
