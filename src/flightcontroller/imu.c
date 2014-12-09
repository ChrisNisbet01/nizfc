#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sensors.h>
#include <sensor_filters.h>
#include <attitude_configuration.h>
#include <vector_rotation.h>
#include <hirestimer.h>
#include <board_alignment.h>
#include <utils.h>

float RollAngle, PitchAngle, Heading;

/*
	When we don't have a magnetometer we can use the gyro to provide a rough heading fix
*/

static float	gyroHeadingVector[3];
static float	estimatedAttitudeVector[3];

uint32_t IMUDelta;
float    fIMUDelta;
float accelerometerValues[3];
float gyroValues[3];
float magnetometerValues[3];
float filteredAccelerometerValues[3];
float filteredGyroValues[3];
float filteredMagnetometerValues[3];

static uint32_t lastIMUTime;
static uint32_t IMUExeTime;

static float calculateHeading(float * headingVector, float roll, float pitch)
{
    float headX;
    float headY;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
	float heading;

    cos_roll = cosf((roll * M_PI)/180.0f);
    sin_roll = sinf((roll * M_PI)/180.0f);
    cos_pitch = cosf((pitch * M_PI)/180.0f);
    sin_pitch = sinf((pitch * M_PI)/180.0f);

    // Tilt compensated magnetic field X component:
    headX = headingVector[1] * sin_roll * sin_pitch + headingVector[0] * cos_pitch + headingVector[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y component:
    headY = headingVector[1] * cos_roll - headingVector[2] * sin_roll;

    // magnetic heading
    heading = atan2f(headY,headX) * 180.0f/M_PI;

	heading = normaliseHeading( heading );

	return heading;
}

static float estimateHeading(float const * const deltaGyroAngle, float roll, float pitch)
{
	float heading;
	vectorRotation_st matrix;

	/* use gyro to determine heading */
	initVectorRotationDegrees( &matrix, deltaGyroAngle[0], deltaGyroAngle[1], deltaGyroAngle[2] );
	applyVectorRotation( &matrix, gyroHeadingVector );
	normaliseVector( gyroHeadingVector, gyroHeadingVector );

	heading = calculateHeading( gyroHeadingVector, roll, pitch );

	return heading;
}

static void estimateAttitude( float const * const deltaGyroAngle, float const * const accelerometerValues, float * roll, float * pitch)
{
    unsigned int axis;
	vectorRotation_st matrix;
    float ComplementaryFilterDivisor = (attitude_configuration[0].complementaryFilterFactor + 1.0f);

	initVectorRotationDegrees( &matrix, deltaGyroAngle[0], deltaGyroAngle[1], deltaGyroAngle[2] );
	applyVectorRotation( &matrix, estimatedAttitudeVector );

    for (axis = 0; axis < 3; axis++)
        estimatedAttitudeVector[axis] = (estimatedAttitudeVector[axis] * attitude_configuration[0].complementaryFilterFactor + accelerometerValues[axis]) / ComplementaryFilterDivisor;

    *roll = (atan2f(estimatedAttitudeVector[1], estimatedAttitudeVector[2]) * 180.0f)/M_PI;
    *pitch = (atan2f(-estimatedAttitudeVector[0], sqrtf(estimatedAttitudeVector[1] * estimatedAttitudeVector[1] + estimatedAttitudeVector[2] * estimatedAttitudeVector[2]))*180.0f)/M_PI;

}

static bool readGyro( sensorCallback_st * sensorCallbacks )
{
	bool gotGyroValues;

	if ( sensorCallbacks->readGyro != NULL && sensorCallbacks->readGyro( sensorCallbacks->gyroCtx, gyroValues ) == true )
	{
		gotGyroValues = true;

		// TODO matrix multiplication to avoid doing two transformations?
		alignVectorToFlightController( gyroValues, noRotation );			// TODO: configurable/per hardware, not cpu
		alignVectorToCraft( gyroValues );

		filterGyroValues( gyroValues, filteredGyroValues );

	}
	else
		gotGyroValues = false;

	return gotGyroValues;
}

static bool readAccelerometer( sensorCallback_st * sensorCallbacks )
{
	bool gotAccValues;

	if ( sensorCallbacks->readAccelerometer != NULL	&& sensorCallbacks->readAccelerometer( sensorCallbacks->accelerometerCtx, accelerometerValues ) == true)
	{
		gotAccValues = true;

		// TODO matrix multiplication to avoid doing two transformations?
		alignVectorToFlightController( accelerometerValues, noRotation );	// TODO: configurable/per hardware, not cpu
		alignVectorToCraft( accelerometerValues );

		filterAccValues( accelerometerValues, filteredAccelerometerValues );
	}
	else
		gotAccValues = false;

	return gotAccValues;
}

static bool readMagnetometer( sensorCallback_st * sensorCallbacks )
{
	bool gotMagnetometerValues;

	if ( sensorCallbacks->readMagnetometer != NULL
		&& sensorCallbacks->readMagnetometer( sensorCallbacks->magnetometerCtx, magnetometerValues ) == true )
	{
		gotMagnetometerValues = true;
		// TODO matrix multiplication to avoid doing two transformations?
		alignVectorToFlightController( magnetometerValues, noRotation );	// TODO: configurable
		alignVectorToCraft( magnetometerValues );

		filterMagValues( magnetometerValues, filteredMagnetometerValues );
	}
	else
		gotMagnetometerValues = false;

	return gotMagnetometerValues;
}

void updateIMU( sensorCallback_st * sensorCallbacks )
{
	bool gotGyroValues;
	bool gotAccValues;
	bool gotMagnetometerValues;
	float deltaGyroAngle[3];

	uint32_t now = micros();
	IMUDelta = now - lastIMUTime;
	lastIMUTime = now;

	/* calculate time in seconds between iterations */
	fIMUDelta = (float)IMUDelta * 1e-6f;

	gotGyroValues = readGyro( sensorCallbacks );
	gotAccValues = readAccelerometer( sensorCallbacks );
	gotMagnetometerValues = readMagnetometer( sensorCallbacks );

	if ( gotGyroValues == true )
	{
		//estimateAttitude( fIMUDelta );
		deltaGyroAngle[0] = filteredGyroValues[0] * fIMUDelta;
		deltaGyroAngle[1] = filteredGyroValues[1] * fIMUDelta;
		deltaGyroAngle[2] = filteredGyroValues[2] * fIMUDelta;

		/* only estimate attitude (used in Angle mode) if gyro and accelerometer both found */
		if ( gotAccValues == true )
			estimateAttitude( deltaGyroAngle, filteredAccelerometerValues, &RollAngle, &PitchAngle );

		if ( gotMagnetometerValues == false )
		{
			Heading = estimateHeading( deltaGyroAngle, RollAngle, PitchAngle );
		}
	}

	/* determine heading from magnetometer */
	if ( gotMagnetometerValues == true )
	{
		Heading = calculateHeading( filteredMagnetometerValues, -RollAngle, -PitchAngle );

		// TODO: apply declination (and renormalise afterwards)
		// TODO: complementary filter with gyro?
	}

	IMUExeTime = micros() - lastIMUTime;

}

float getIMUExeTime( void )
{
	return IMUExeTime;
}

void initGyroHeadingVector( void )
{
	/* current attitude is used as the starting point for gyro based heading */
	gyroHeadingVector[0] = 1.0f;
	gyroHeadingVector[1] = 0.0f;
	gyroHeadingVector[2] = 0.0f;
}

void initIMUTime( uint32_t time )
{
	lastIMUTime = time;
}
