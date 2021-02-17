/*
 * lib_kontrol.c
 *
 *  Created on: Jan 25, 2021
 *      Author: Prasetyo Wibowo (Sibernetika)
 */

#include <math.h>
#include <stdlib.h>

#include "lib_kontrol.h"

Konstanta_PID_t kPID[4];

const Konstanta_PID_t kPID_0 = { 0, 0.65, 0.5, 0.05, 1.2, 0.5, 0.8, 0 };
const Konstanta_PID_t kPID_1 = { 1, 0.1, 0.07, 1.0, 0.8, 4, 0.3, 0 };
const Konstanta_PID_t kPID_2 = { 2, 0.05, 0.03, 0.0, 0.0, 0.0, 0.0, 2 };
const Konstanta_PID_t kPID_3 = { 3, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0 };

// PID Params
//float kp1 = 1, kp2 = 1;
//float kd1 = 0, kd2 = 0;
//float ki1 = 0, ki2 = 0;
//int THRESHOLD_PX = 1;

float Torq1, Torq2;
float scale = 0.00032;
float q_acc0, q_acc1;
float theta1, theta2;
float theta1_d, theta2_d;
float g0 = 9.81;
int eq = 1;

static float toDeg(float a)
{
	return a * 180 / M_PI;
}

static float toRad(float a)
{
	return a * M_PI / 180;
}

static float wrapVal(float a, float b)
{
	if (a < -b) {
		return -b;
	}
	else if (a > b) {
		return b;
	}
	else {
		return a;
	}
}

void Kontrol_init()
{
	q_acc0 = 0;
	q_acc1 = 0;
}

void Kontrol_Konstanta_init()
{
	kPID[0] = kPID_0;
	kPID[1] = kPID_1;
	kPID[2] = kPID_2;
	kPID[3] = kPID_3;
}

void Kontrol_Set(int zoomLevel, float constPID[6], int thresholdPx)
{
	Konstanta_PID_t newPID;

	if (zoomLevel < 0)
		zoomLevel = 0;
	else if (zoomLevel > 3)
		zoomLevel = 3;

	newPID.zoomLevel = zoomLevel;
	newPID.kp1 = constPID[0];
	newPID.kp2 = constPID[1];
	newPID.kd1 = constPID[2];
	newPID.kd2 = constPID[3];
	newPID.ki1 = constPID[4];
	newPID.ki2 = constPID[5];
	newPID.thresholdPX = thresholdPx;

	kPID[zoomLevel] = newPID;
}

void Kontrol_Get(int zoomLevel, Konstanta_PID_t *constPID)
{
	if (zoomLevel < 0)
		zoomLevel = 0;
	else if (zoomLevel > 3)
		zoomLevel = 3;

	*constPID = kPID[zoomLevel];
}

void Kontrol_CalcQDot(int zoomLevel, float qaz, float qev, float qdaz, float qdev, int dx, int dy,
		float* qd_sp_az, float* qd_sp_ev)
{
	theta1 = toRad(qaz);
	theta2 = toRad(qev);
	theta1_d = toRad(qdaz);
	theta2_d = toRad(qdev);

	// Gain Schedule
	if (zoomLevel < 0)
		zoomLevel = 0;
	else if (zoomLevel > 3)
		zoomLevel = 3;

	Konstanta_PID_t *pid = &kPID[zoomLevel];
	float kp1 = pid->kp1;
	float kp2 = pid->kp2;
	float kd1 = pid->kd1;
	float kd2 = pid->kd2;
	float ki1 = pid->ki1;
	float ki2 = pid->ki2;
	int THRESHOLD_PX = pid->thresholdPX;

	if (eq == 1) {
		scale = 0.1;
		Torq1 = toRad(kp1 * dx - kd1 * theta1_d + ki1 * q_acc0);
		Torq2 = toRad(kp2 * dy - kd2 * theta2_d + ki2 * q_acc1);
		if (abs(dx) <= THRESHOLD_PX) {
			Torq1 = 0;
		}
		if (abs(dy) <= THRESHOLD_PX) {
			Torq2 = 0;
		}
	}
	else if (eq == 2) {
		Torq1 = (dx * kp1 - kd1 * theta1_d + ki1 * q_acc0)
				* (-(-9.8894180600000006f * sinf(theta1) - 0.037655269999999998f * cosf(theta1))
						* sinf(theta1)
						- (-5.7034281900000003f * sinf(theta1) - 0.39765412f * cosf(theta1))
								* sinf(theta1)
						- (-0.39765412f * sinf(theta1) - 5.5533192599999994f * cosf(theta1))
								* cosf(theta1)
						+ 0.017725477344000002f
								* powf(-0.31109324758842444f * sinf(theta1) - cosf(theta1), 2)
						- (-0.037655269999999991f * sinf(theta1) - 4.09682367f * cosf(theta1))
								* cosf(theta1)
						+ 0.017725477344000002f
								* powf(sinf(theta1) - 0.31109324758842444f * cosf(theta1), 2)
						+ 1.8409072500000003f
								* powf(
										0.15155555555555555f * sinf(theta1) * sinf(theta2)
												- 0.061851851851851852f * sinf(theta1)
														* cosf(theta2) - sinf(theta1)
												- 0.31651851851851853f * cosf(theta1), 2)
						+ 1.8409072500000003f
								* powf(
										0.31651851851851853f * sinf(theta1)
												+ 0.15155555555555555f * sinf(theta2) * cosf(theta1)
												- 0.061851851851851852f * cosf(theta1)
														* cosf(theta2) - cosf(theta1), 2))
				+ (dy * kp2 - kd2 * theta2_d + ki2 * q_acc1)
						* (101.01000000000001f
								* (0.0083499999999999998f * sinf(theta1) * sinf(theta2)
										+ 0.020459999999999999f * sinf(theta1) * cosf(theta2))
								* (0.042729999999999997f * sinf(theta1)
										+ 0.020459999999999999f * sinf(theta2) * cosf(theta1)
										- 0.0083499999999999998f * cosf(theta1) * cosf(theta2)
										- 0.13500000000000001f * cosf(theta1))
								+ 101.01000000000001f
										* (-0.0083499999999999998f * sinf(theta2) * cosf(theta1)
												- 0.020459999999999999f * cosf(theta1)
														* cosf(theta2))
										* (0.020459999999999999f * sinf(theta1) * sinf(theta2)
												- 0.0083499999999999998f * sinf(theta1)
														* cosf(theta2)
												- 0.13500000000000001f * sinf(theta1)
												- 0.042729999999999997f * cosf(theta1))
								- (-9.8894180600000006f * sinf(theta1)
										- 0.037655269999999998f * cosf(theta1)) * sinf(theta1)
								- (-0.037655269999999991f * sinf(theta1)
										- 4.09682367f * cosf(theta1)) * cosf(theta1));

		Torq2 = 101.01000000000001f * g0
				* (-0.020459999999999999f * sinf(theta2) + 0.0083499999999999998f * cosf(theta2))
				+ (dx * kp1 - kd1 * theta1_d + ki1 * q_acc0)
						* (101.01000000000001f
								* (0.0083499999999999998f * sinf(theta1) * sinf(theta2)
										+ 0.020459999999999999f * sinf(theta1) * cosf(theta2))
								* (0.042729999999999997f * sinf(theta1)
										+ 0.020459999999999999f * sinf(theta2) * cosf(theta1)
										- 0.0083499999999999998f * cosf(theta1) * cosf(theta2)
										- 0.13500000000000001f * cosf(theta1))
								+ 101.01000000000001f
										* (-0.0083499999999999998f * sinf(theta2) * cosf(theta1)
												- 0.020459999999999999f * cosf(theta1)
														* cosf(theta2))
										* (0.020459999999999999f * sinf(theta1) * sinf(theta2)
												- 0.0083499999999999998f * sinf(theta1)
														* cosf(theta2)
												- 0.13500000000000001f * sinf(theta1)
												- 0.042729999999999997f * cosf(theta1))
								- (-9.8894180600000006f * sinf(theta1)
										- 0.037655269999999998f * cosf(theta1)) * sinf(theta1)
								- (-0.037655269999999991f * sinf(theta1)
										- 4.09682367f * cosf(theta1)) * cosf(theta1))
				+ (dy * kp2 - kd2 * theta2_d + ki2 * q_acc1)
						* (0.042283957716000013f
								* powf(
										0.40811339198435975f * sinf(theta1) * sinf(theta2)
												+ sinf(theta1) * cosf(theta2), 2)
								+ 0.042283957716000013f
										* powf(
												-0.40811339198435975f * sinf(theta2) * cosf(theta1)
														- cosf(theta1) * cosf(theta2), 2)
								- (-9.8894180600000006f * sinf(theta1)
										- 0.037655269999999998f * cosf(theta1)) * sinf(theta1)
								- (-0.037655269999999991f * sinf(theta1)
										- 4.09682367f * cosf(theta1)) * cosf(theta1)
								+ 0.042283957716000013f
										* powf(sinf(theta2) - 0.40811339198435975f * cosf(theta2),
												2));
	}

	Torq1 = Torq1 * scale;
	Torq2 = Torq2 * scale;

	q_acc0 += dx;
	q_acc1 += dy;
	q_acc0 = wrapVal(q_acc0, 10000);
	q_acc1 = wrapVal(q_acc1, 10000);

	if (abs(dx) <= THRESHOLD_PX) {
		q_acc0 = 0;
	}
	if (abs(dy) <= THRESHOLD_PX) {
		q_acc1 = 0;
	}

	*qd_sp_az = wrapVal(toDeg(Torq1), 70.0f);
	*qd_sp_ev = wrapVal(toDeg(Torq2), 90.0f);
}
