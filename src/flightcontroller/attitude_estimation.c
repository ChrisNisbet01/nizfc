/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "attitude_estimation.h"
#include "kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

/*
	Ported from https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
*/

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RAD_TO_DEG	(180.0f/M_PI)

static Kalman kalmanX;
static Kalman kalmanY;

static float CFX, CFY;

void init_attitude_estimation( IMU_DATA_ST *pdata, float CF_X_factor, float CF_Y_factor )
{
	memset( pdata, 0, sizeof( *pdata ) );
	KalmanInit( &kalmanX );
	KalmanInit( &kalmanY );
    kalmanX.setQangle(&kalmanX, 0.001f);
    kalmanX.setQbias(&kalmanX, 0.003f);
    kalmanX.setRmeasure(&kalmanX, 0.03f);
    kalmanY.setQangle(&kalmanY, 0.001f);
    kalmanY.setQbias(&kalmanY, 0.003f);
    kalmanY.setRmeasure(&kalmanY, 0.03f);


	CFX = CF_X_factor;
	CFY = CF_Y_factor;
}

void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroXrate, float gyroYrate, float accX, float accY, float accZ )
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	float roll  = -atan2f(accY, accZ) * RAD_TO_DEG;
	float pitch = -atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	float roll  = -atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	float pitch = -atan2f(-accX, accZ) * RAD_TO_DEG;
#endif
	float rotationX, rotationY;

#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90.0f && pdata->kalAngleX > 90.0f) || (roll > 90.0f && pdata->kalAngleX < -90.0f))
	{
		kalmanX.setAngle(&kalmanX, roll);
		pdata->compAngleX = roll;
		pdata->kalAngleX = roll;
		pdata->gyroXangle = roll;
	}
	else
		pdata->kalAngleX = kalmanX.getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (fabsf(pdata->kalAngleX) > 90.0f)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
	pdata->kalAngleY = kalmanY.getAngle(&kalmanY, pitch, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && pdata->kalAngleY > 90.0f) || (pitch > 90 && pdata->kalAngleY < -90.0f))
	{
	    kalmanY.setAngle(&kalmanY, pitch);
	    pdata->compAngleY = pitch;
	    pdata->kalAngleY = pitch;
	    pdata->gyroYangle = pitch;
	}
	else
    	pdata->kalAngleY = kalmanY.getAngle(&kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (fabsf(pdata->kalAngleY) > 90.0f)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
	pdata->kalAngleX = kalmanX.getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
	rotationX = gyroXrate * dt;
	rotationY = gyroYrate * dt;

	pdata->gyroXangle += rotationX; // Calculate gyro angle without any filter
	pdata->gyroYangle += rotationY;

	pdata->compAngleX = (1.0f-CFX) * (pdata->compAngleX + rotationX) + CFX * roll; // Calculate the angle using a Complementary filter
	pdata->compAngleY = (1.0f-CFY) * (pdata->compAngleY + rotationY) + CFY * pitch;

	pdata->compAngleX2 += rotationX;
	pdata->compAngleY2 += rotationY;

	pdata->compAngleX2 = (pdata->compAngleX2 * 600 + roll) / 601.0f;
	pdata->compAngleY2 = (pdata->compAngleY2 * 600 + pitch) / 601.0f;


	pdata->roll = roll;
	pdata->pitch = pitch;

	// Reset the gyro angle when it has drifted too much
	if (pdata->gyroXangle < -180.0f || pdata->gyroXangle > 180.0f)
		pdata->gyroXangle = pdata->compAngleX;
	if (pdata->gyroYangle < -180.0f || pdata->gyroYangle > 180.0f)
		pdata->gyroYangle = pdata->compAngleY;
}
