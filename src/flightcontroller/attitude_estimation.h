#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

/* IMU Data */
typedef struct IMU_DATA_ST
{
	float roll;		    // angle from accelerometer only */
	float pitch;

	float gyroRollAngle; 	// Angle calculated using the gyro only
	float gyroPitchAngle;

	float compRollAngle; 	// Calculated angle using a complementary filter
	float compPitchAngle;

} IMU_DATA_ST;


void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroRollRate, float gyroPitchRate, float accX, float accY, float accZ );

#endif /*  __ATTITUDE_ESTIMATION_H__ */
