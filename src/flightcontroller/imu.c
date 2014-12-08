#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sensors.h>
#include <sensor_filters.h>
#include <attitude_estimation.h>
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

uint32_t IMUDelta;
float    fIMUDelta;
IMU_DATA_ST imu_data;
float accelerometerValues[3];
float gyroValues[3];
float magnetometerValues[3];
float filteredAccelerometerValues[3];
float filteredGyroValues[3];
float filteredMagnetometerValues[3];

static uint32_t lastIMUTime;
static uint32_t IMUExeTime;

static float calculateHeading(float *headingVector, float roll, float pitch)
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

	// TODO: apply declination

	heading = normaliseHeading( heading );

	return heading;
}

float getIMUExeTime( void )
{
	return IMUExeTime;
}

void initGyroHeadingVector( void )
{
	/* start out pointing straight ahead */
	gyroHeadingVector[0] = 1.0f;
	gyroHeadingVector[1] = 0.0f;
	gyroHeadingVector[2] = 0.0f;
}

static void estimateAttitude( float dT )
{

    do_attitude_estimation( &imu_data,
    				dT,
    				filteredGyroValues[1],	// TODO: rotate vectors appropriately before calling this
    				-filteredGyroValues[0],
    				filteredAccelerometerValues[0],
    				filteredAccelerometerValues[1],
    				filteredAccelerometerValues[2] );

    RollAngle = imu_data.compRollAngle;
    PitchAngle = imu_data.compPitchAngle;
}

bool readGyro( sensorCallback_st * sensorCallbacks )
{
	bool gotGyroValues;

	if ( sensorCallbacks->readGyro != NULL && sensorCallbacks->readGyro( sensorCallbacks->gyroCtx, gyroValues ) == true )
	{
		gotGyroValues = true;

		// TODO matrix multiplication to avoid doing two transformations?
#if defined(STM32F30X)
		alignVectorsToFlightController( gyroValues, noRotation );			// TODO: configurable/per hardware, not cpu
#elif defined(STM32F10X)
		alignVectorsToFlightController( gyroValues, clockwise270Degrees );			// TODO: configurable/per hardware, not cpu
#endif
		alignVectorsToCraft( gyroValues );

		filterGyroValues( gyroValues, filteredGyroValues );

	}
	else
		gotGyroValues = false;

	return gotGyroValues;
}

bool readAccelerometer( sensorCallback_st * sensorCallbacks )
{
	bool gotAccValues;

	if ( sensorCallbacks->readAccelerometer != NULL	&& sensorCallbacks->readAccelerometer( sensorCallbacks->accelerometerCtx, accelerometerValues ) == true)
	{
		gotAccValues = true;

		// TODO matrix multiplication to avoid doing two transformations?
#if defined(STM32F30X)
		alignVectorsToFlightController( accelerometerValues, noRotation );	// TODO: configurable/per hardware, not cpu
#elif defined(STM32F10X)
		alignVectorsToFlightController( accelerometerValues, clockwise180Degrees );	// TODO: configurable/per hardware, not cpu
#endif
		alignVectorsToCraft( accelerometerValues );

		filterAccValues( accelerometerValues, filteredAccelerometerValues );
	}
	else
		gotAccValues = false;

	return gotAccValues;
}

bool readMagnetometer( sensorCallback_st * sensorCallbacks )
{
	bool gotMagnetometerValues;

	if ( sensorCallbacks->readMagnetometer != NULL
		&& sensorCallbacks->readMagnetometer( sensorCallbacks->magnetometerCtx, magnetometerValues ) == true )
	{
		gotMagnetometerValues = true;
		// TODO matrix multiplication to avoid doing two transformations?
		alignVectorsToFlightController( magnetometerValues, noRotation );	// TODO: configurable
		alignVectorsToCraft( magnetometerValues );

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
	uint32_t now = micros();
	IMUDelta = now - lastIMUTime;
	lastIMUTime = now;

	/* calculate time in seconds between iterations */
	fIMUDelta = (float)IMUDelta * 1e-6f;

	gotGyroValues = readGyro( sensorCallbacks );
	gotAccValues = readAccelerometer( sensorCallbacks );
	gotMagnetometerValues = readMagnetometer( sensorCallbacks );

	/* only estimate attitude (used in Angle mode) if gyro and accelerometer both found */
	if ( gotGyroValues == true && gotAccValues == true )
	{
		estimateAttitude( fIMUDelta );
	}

	/* determine heading */
	if ( gotMagnetometerValues == true )
	{
		Heading = calculateHeading( filteredMagnetometerValues, -RollAngle, -PitchAngle );
	}
	else if ( gotGyroValues == true )
	{
		float deltaGyroAngle[3];
		vectorRotation_st matrix;

		deltaGyroAngle[0] = gyroValues[0] * fIMUDelta;
		deltaGyroAngle[1] = gyroValues[1] * fIMUDelta;	// XXX fix up rotation here and in estimateAttitude and above
		deltaGyroAngle[2] = gyroValues[2] * fIMUDelta;

		/* use gyro to determine heading */
		initVectorRotationDegrees( &matrix, deltaGyroAngle[0], deltaGyroAngle[1], deltaGyroAngle[2] );
		applyVectorRotation( &matrix, gyroHeadingVector );
		normaliseVector( gyroHeadingVector, gyroHeadingVector );
#if defined(STM32F30X)
		Heading = calculateHeading( gyroHeadingVector, -RollAngle, -PitchAngle );
#elif defined(STM32F10X)
		Heading = calculateHeading( gyroHeadingVector, -PitchAngle, RollAngle );
#endif
	}

	IMUExeTime = micros() - lastIMUTime;

}

void initIMUTime( uint32_t time )
{
	lastIMUTime = time;
}
