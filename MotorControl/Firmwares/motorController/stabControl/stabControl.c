/*
 * stabControl.c
 *
 *  Created on: Feb 10, 2021
 *      Author: miftakur
 */

#include <stdbool.h>
#include <math.h>
#include "stabControl.h"

typedef struct
{
	float f;
	int32_t i32;
	uint32_t u32;
	uint8_t b[4];
} Union_u;

/* PID Params */
#define kp1_v 10.5f
#define kp2_v 10.5f
#define kd1_v 0.0f
#define kd2_v 0.0f
#define ki1_v 0.01f
#define ki2_v 0.05f

/* Model States */
float x0 = -90.0f * M_PI / 180.0f;
float x1 = -90.0f * M_PI / 180.0f;
float x2 = 0.0f;
float theta1 = 90.0f * M_PI / 180.0f;
float theta2 = 0.0f;
float theta1_d, theta2_d;
float q_acc1, q_acc2;
float e_q1, e_q2;
float q_des_now_1, q_des_now_2;
int first_start = 1;

float x0_target = -90.0f * M_PI / 180.0f;
float x1_target = -90.0f * M_PI / 180.0f;
float x2_target = 0.0f;
float theta1_target = 90.0f * M_PI / 180.0f;
float theta2_target = 0.0f;
bool update_target = false;

float Torque1 = 0.0f;
float Torque2 = 0.0f;
float posl_ee[3] = { 0.0f, 0.0f, 0.0f };
float A4Iinv[4][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0.135, 0, 0, 1 } };
float target_pos[4] = { 0, 0, 0, 1 };
float targetl_ee[3], targetl_ws[3];
float dpitch, dyaw;

float backup_val[6] = { kp1_v, kp2_v, kd1_v, kd2_v, ki1_v, ki2_v };
float kp1 = kp1_v, kp2 = kp2_v;
float kd1 = kd1_v, kd2 = kd2_v;
float ki1 = ki1_v, ki2 = ki2_v;
int save_pid_q = 0;
int debugIMUcount = 0;
int i = 0;

Union_u sp_send1;
Union_u sp_send2;

float offset0 = 0.0f, offset1 = 0.0f;

/* Private Functions */
static void calc_target();
static void calc_torques();
static void forward_kinematics();

/* Vector Utils */
static float dot(float, float, float, float, float, float);
static float norm(float, float, float);
static float vectorAngle(float, float, float, float, float, float);
static float vectorAngle2(float, float, float, float, float, float);
static float toDeg(float);
static float toRad(float);
static int sign(float);
static float wrapAngle(float);
static float wrapVal(float, float);

void stab_calculate(int mode, float pitch, float roll, float q_az, float q_ev, float qd_az,
		float qd_ev, float *sp1, float *sp2)
{
	theta1 = wrapAngle(toRad(-q_az + 90));
	theta2 = wrapAngle(toRad(q_ev));
	theta1_d = toRad(-qd_az);
	theta2_d = toRad(qd_ev);
	x0 = wrapAngle(toRad(pitch - 90));
	x1 = wrapAngle(toRad(roll - 90));

	if (fabs(-qd_az) < 0.061) {
		theta1_d = 0;
	}
	if (fabs(qd_ev) < 0.061) {
		theta2_d = 0;
	}

	static uint32_t processTimer = 0;

	if (HAL_GetTick() >= processTimer) {
		processTimer = HAL_GetTick() + 5;
		if (!mode) {
			*sp1 = 0.0f;
			*sp2 = 0.0f;
		}
		else if (mode) {
			forward_kinematics();  // OK
			calc_torques();  // OK
			*sp1 = sp_send1.f;
			*sp2 = sp_send2.f;

		}

	}
}

void stab_reset()
{
	q_acc1 = 0;
	q_acc2 = 0;
	dpitch = 0;
	dyaw = 0;
	e_q1 = 0;
	e_q2 = 0;
	first_start = 1;
}

void stab_updateSetpoint(float dq_az, float dq_ev)
{
	theta1_target += toRad(-dq_az) * 0.1f;  //10 Hz
	theta1_target += toRad(dq_ev) * 0.1f;
	update_target = true;
}

static void calc_target()
{
	target_pos[0] = x2 * sinf(x1_target) * cosf(x0_target)
			+ 1.0965
					* (-sinf(theta1_target) * sinf(x0_target)
							+ cosf(theta1_target) * cosf(x0_target) * cosf(x1_target))
					* cosf(theta2_target) + 0.135 * sinf(theta1_target) * sinf(x0_target)
			+ 1.0965 * sinf(theta2_target) * sinf(x1_target) * cosf(x0_target)
			+ 0.4705 * sinf(x1_target) * cosf(x0_target)
			- 0.135 * cosf(theta1_target) * cosf(x0_target) * cosf(x1_target);
	target_pos[1] = -x2 * cosf(x1_target) - 1.0965 * sinf(theta2_target) * cosf(x1_target)
			+ 1.0965 * sinf(x1_target) * cosf(theta1_target) * cosf(theta2_target)
			- 0.135 * sinf(x1_target) * cosf(theta1_target) - 0.4705 * cosf(x1_target);
	target_pos[2] = x2 * sinf(x0_target) * sinf(x1_target)
			+ 1.0965
					* (sinf(theta1_target) * cosf(x0_target)
							+ sinf(x0_target) * cosf(theta1_target) * cosf(x1_target))
					* cosf(theta2_target) - 0.135 * sinf(theta1_target) * cosf(x0_target)
			+ 1.0965 * sinf(theta2_target) * sinf(x0_target) * sinf(x1_target)
			+ 0.4705 * sinf(x0_target) * sinf(x1_target)
			- 0.135 * sinf(x0_target) * cosf(theta1_target) * cosf(x1_target);
}

static void forward_kinematics()
{
	//1. Calc Target Pos (Frame I)
	if (first_start == 1) {
		stab_reset();
		theta1_target = theta1;
		theta2_target = theta2;
		x0_target = x0;
		x1_target = x1;
		calc_target();
		first_start = 0;
	}

	if (update_target) {
		calc_target();
		update_target = false;
	}

	//2. Calc Pos EE (Frame L)
	posl_ee[0] = 1.0965 * cosf(theta2);
	posl_ee[1] = 1.0965 * sinf(theta2);

	//3, Calc Target Pos (Frame L)
	A4Iinv[0][0] = -sinf(theta1) * sinf(x0) + cosf(theta1) * cosf(x0) * cosf(x1);
	A4Iinv[1][0] = sinf(x1) * cosf(theta1);
	A4Iinv[2][0] = sinf(theta1) * cosf(x0) + sinf(x0) * cosf(theta1) * cosf(x1);
	A4Iinv[0][1] = sinf(x1) * cosf(x0);
	A4Iinv[1][1] = -cosf(x1);
	A4Iinv[2][1] = sinf(x0) * sinf(x1);
	A4Iinv[3][1] = -x2 - 0.4705;
	A4Iinv[0][2] = sinf(theta1) * cosf(x0) * cosf(x1) + sinf(x0) * cosf(theta1);
	A4Iinv[1][2] = sinf(theta1) * sinf(x1);
	A4Iinv[2][2] = sinf(theta1) * sinf(x0) * cosf(x1) - cosf(theta1) * cosf(x0);

	for ( int i = 0; i < 3; i++ ) {  //baris
		targetl_ee[i] = 0;
		for ( int k = 0; k < 4; k++ ) {  //kolom
			targetl_ee[i] += A4Iinv[k][i] * target_pos[k];
		}
	}

	targetl_ws[0] = targetl_ee[0] / norm(targetl_ee[0], targetl_ee[1], targetl_ee[2]) * 1.0965;
	targetl_ws[1] = targetl_ee[1] / norm(targetl_ee[0], targetl_ee[1], targetl_ee[2]) * 1.0965;
	targetl_ws[2] = targetl_ee[2] / norm(targetl_ee[0], targetl_ee[1], targetl_ee[2]) * 1.0965;

	if (sign(posl_ee[0]) != sign(targetl_ws[0])) {
		dyaw = wrapAngle(vectorAngle2(posl_ee[0], 0, posl_ee[2], targetl_ws[0], 0, targetl_ws[2]));
	}
	else {
		dyaw = wrapAngle(-vectorAngle(0, 0, 1, targetl_ws[0], 0, targetl_ws[2]));
	}

	dpitch = wrapAngle(
			vectorAngle(0, 1, 0, targetl_ws[0], targetl_ws[1], 0)
					- vectorAngle(0, 1, 0, posl_ee[0], posl_ee[1], 0));
	q_des_now_1 = theta1 + dyaw;
	q_des_now_2 = theta2 + dpitch;
}

static void calc_torques()
{
	e_q1 = wrapVal(q_des_now_1 - theta1, toRad(3.2));
	e_q2 = wrapVal(q_des_now_2 - theta2, toRad(3.2));

	// Reset Integral Every Time Error Crossed zero
	if (fabs(e_q1) <= 0.05) {
		q_acc1 = 0;
	}
	if (fabs(e_q2) <= 0.05) {
		q_acc2 = 0;
	}

	Torque1 = (kp1 * e_q1 - kd1 * theta1_d + ki1 * q_acc1);
	Torque2 = (kp2 * e_q2 - kd2 * theta2_d + ki2 * q_acc2);

	sp_send1.f = wrapVal(-toDeg(Torque1), 32);
	sp_send2.f = wrapVal(toDeg(Torque2), 32);  //90
//  sp_send2.f = LPF.input(sp_send2.f);
	if (fabs(sp_send1.f) <= 0.001) {
		sp_send1.f = 0;
	}
	if (fabs(sp_send2.f) <= 0.001) {
		sp_send2.f = 0;
	}

	q_acc1 += e_q1;
	q_acc2 += e_q2;
	q_acc1 = wrapVal(q_acc1, 30);
	q_acc2 = wrapVal(q_acc2, 30);
}

/* Vector Utils -------------------------*/
static float dot(float ax, float ay, float az, float bx, float by, float bz)
{
	float dotres = 0.0f;
	dotres += ax * bx;
	dotres += ay * by;
	dotres += az * bz;
	return dotres;
}

static float norm(float x, float y, float z)
{
	float acc = 0.0f;
	acc += x * x;
	acc += y * y;
	acc += z * z;
	return sqrt(acc);
}

static float vectorAngle(float ax, float ay, float az, float bx, float by, float bz)
{
	float dotprod = dot(ax, ay, az, bx, by, bz);
	return asin(dotprod / (norm(ax, ay, az) * norm(bx, by, bz)));
}

static float vectorAngle2(float ax, float ay, float az, float bx, float by, float bz)
{
	float dotprod = dot(ax, ay, az, bx, by, bz);
	return acos(dotprod / (norm(ax, ay, az) * norm(bx, by, bz)));
}

static float toDeg(float a)
{
	return a * 180.0f / M_PI;
}

static float toRad(float a)
{
	return a * M_PI / 180.0f;
}

static int sign(float a)
{
	if (a != 0) {
		return a / fabs(a);
	}
	else {
		return 1;
	}
}

static float wrapAngle(float rad)
{
	float angle = fmod(toDeg(rad), (float) (sign(rad) * 360));
	if (angle > 180.0f) {
		angle = -(360.0f - angle);
	}
	else if (angle < -180.0f) {
		angle = (360.0f + angle);
	}
	else {
		angle = angle;
	}
	return toRad(angle);
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
