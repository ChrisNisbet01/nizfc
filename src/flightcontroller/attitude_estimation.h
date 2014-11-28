#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

/* IMU Data */
typedef struct IMU_DATA_ST
{
	float roll;		    // angle from accelerometer only */
	float pitch;

	float gyroXangle; 	// Angle calculated using the gyro only
	float gyroYangle;

	float compAngleX; 	// Calculated angle using a complementary filter
	float compAngleY;

} IMU_DATA_ST;


void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroXrate, float gyroYrate, float accX, float accY, float accZ );

#endif /*  __ATTITUDE_ESTIMATION_H__ */
