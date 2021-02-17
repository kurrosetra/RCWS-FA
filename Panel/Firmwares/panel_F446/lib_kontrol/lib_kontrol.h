/*
 * lib_kontrol.h
 *
 *  Created on: Jan 25, 2021
 *      Author: Prasetyo Wibowo (Sibernetika)
 */

#ifndef LIB_KONTROL_H_
#define LIB_KONTROL_H_

/*
 * Sample Usage:
 *
 *
 * int main() {
 * 	float qd1 = 0;
 * 	float qd2 = 0;
 * 	CalcQDot(1, 90.0, 0.0, 0.0, 0.0, 1, 1, &qd1, &qd2);
 * 	cout << qd1 << ", " << qd2;
 * }
 *
 * qd1 = 0.1f
 * qd2 = 0.1f
 *
 */

typedef struct
{
	int zoomLevel;
	float kp1;
	float kp2;
	float kd1;
	float kd2;
	float ki1;
	float ki2;
	int thresholdPX;
} Konstanta_PID_t;

void Kontrol_init();
void Kontrol_Konstanta_init();
void Kontrol_CalcQDot(int zoomLevel, float qaz, float qev, float qdaz, float qdev, int dx, int dy,
		float* qd_sp_az, float* qd_sp_ev);
void Kontrol_Set(int zoomLevel, float constPID[6], int thresholdPx);
void Kontrol_Get(int zoomLevel, Konstanta_PID_t *constPID);

#endif /* LIB_KONTROL_H_ */
