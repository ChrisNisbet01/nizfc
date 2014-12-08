#include <stdint.h>
#include <string.h>
#include <math.h>
#include <filter.h>
#include <attitude_configuration.h>
#include "attitude_estimation.h"

#define RAD_TO_DEG	(180.0f/M_PI)

void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroRollRate, float gyroPitchRate, float accX, float accY, float accZ )
{
	float roll  = atan2f(accY, accZ) * RAD_TO_DEG;
	float pitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	float rotationX, rotationY;

	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90.0f && pdata->compRollAngle > 90.0f) || (roll > 90.0f && pdata->compRollAngle < -90.0f))
	{
		pdata->compRollAngle = roll;
		pdata->gyroRollAngle = roll;
	}

	if (fabsf(pdata->compRollAngle) > 90.0f)
		gyroPitchRate = -gyroPitchRate; // Invert rate, so it fits the restricted accelerometer reading

	/* determine rotation for this period */
	rotationX = gyroRollRate * dt;
	rotationY = gyroPitchRate * dt;

	/* integrate it */
	pdata->gyroRollAngle += rotationX;
	pdata->gyroPitchAngle += rotationY;

	/* Reset the gyro angle when it has drifted too much */
	if (pdata->gyroRollAngle < -180.0f || pdata->gyroRollAngle > 180.0f)
		pdata->gyroRollAngle = pdata->compRollAngle;
	if (pdata->gyroPitchAngle < -180.0f || pdata->gyroPitchAngle > 180.0f)
		pdata->gyroPitchAngle = pdata->compPitchAngle;


	// Calculate the angle using a Complementary filter
	pdata->compRollAngle = filterValue( pdata->compRollAngle + rotationX, roll, attitude_configuration[0].roll_lpf );
	pdata->compPitchAngle = filterValue( pdata->compPitchAngle + rotationY, pitch, attitude_configuration[0].pitch_lpf );

	pdata->roll = roll;
	pdata->pitch = pitch;

}

