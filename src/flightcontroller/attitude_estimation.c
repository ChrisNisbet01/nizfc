#include <stdint.h>
#include <string.h>
#include <math.h>
#include <filter.h>
#include "attitude_estimation.h"

#define RAD_TO_DEG	(180.0f/M_PI)

void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroXrate, float gyroYrate, float accX, float accY, float accZ )
{
	float roll  = -atan2f(accY, accZ) * RAD_TO_DEG;
	float pitch = -atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	float rotationX, rotationY;

	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90.0f && pdata->compAngleX > 90.0f) || (roll > 90.0f && pdata->compAngleX < -90.0f))
	{
		pdata->compAngleX = roll;
		pdata->gyroXangle = roll;
	}

	if (fabsf(pdata->compAngleX) > 90.0f)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading

	/* determine rotation for this period */
	rotationX = gyroXrate * dt;
	rotationY = gyroYrate * dt;

	/* integrate it */
	pdata->gyroXangle += rotationX;
	pdata->gyroYangle += rotationY;

	/* Reset the gyro angle when it has drifted too much */
	if (pdata->gyroXangle < -180.0f || pdata->gyroXangle > 180.0f)
		pdata->gyroXangle = pdata->compAngleX;
	if (pdata->gyroYangle < -180.0f || pdata->gyroYangle > 180.0f)
		pdata->gyroYangle = pdata->compAngleY;


	// TODO: configurable
	// Calculate the angle using a Complementary filter
	pdata->compAngleX = filterValue( pdata->compAngleX + rotationX, roll, 200 );
	pdata->compAngleY = filterValue( pdata->compAngleY + rotationY, pitch, 200 );

	pdata->roll = roll;
	pdata->pitch = pitch;

}

