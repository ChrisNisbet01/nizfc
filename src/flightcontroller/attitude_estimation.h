#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

/* IMU Data */
typedef struct IMU_DATA_ST
{
	float	roll;		// angle from accelerometer only */
	float	pitch;

	float gyroXangle; 	// Angle calculated using the gyro only
	float gyroYangle;

	float compAngleX; 	// Calculated angle using a complementary filter
	float compAngleY;

	float compAngleX2; 	// Calculated angle using a complementary filter
	float compAngleY2;

	float kalAngleX; 	// Calculated angle using a Kalman filter
	float kalAngleY;
} IMU_DATA_ST;


void init_attitude_estimation( IMU_DATA_ST *pdata, float CF_X_factor, float CF_Y_factor );
void do_attitude_estimation( IMU_DATA_ST *pdata, float dt, float gyroXrate, float gyroYrate, float accX, float accY, float accZ );

#endif /*  __ATTITUDE_ESTIMATION_H__ */
