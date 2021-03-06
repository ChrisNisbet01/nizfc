#ifndef __IMU_H__
#define __IMU_H__

#include <sensors.h>

void updateIMU( sensorCallback_st * sensorCallbacks );
void initIMUTime( uint32_t time );
void initGyroHeadingVector( void );
float getIMUExeTime( void );

extern float RollAngle, PitchAngle, Heading;
extern float filteredAccelerometerValues[3];
extern float filteredGyroValues[3];
extern float filteredMagnetometerValues[3];
extern float fIMUDelta;
extern uint32_t IMUDelta;
extern float accelerometerValues[3];
extern float gyroValues[3];
extern float magnetometerValues[3];

#endif /* __IMU_H__ */

