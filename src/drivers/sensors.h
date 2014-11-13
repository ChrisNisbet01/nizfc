#ifndef __SENSORS_H__
#define __SENSORS_H__

typedef float temperature_sensor_t;
typedef float accelerometer_sensor_t;
typedef float magnetometer_sensor_t;
typedef float gyrometer_sensor_t;

/* callbacks into the sensor hardware for the various data points */
typedef bool (*readTempFn)( void * pv, temperature_sensor_t * temperature );
typedef bool (*readAccelerometerFn)( void * pv, accelerometer_sensor_t *accelerometer );
typedef bool (*readMagnetometerFn)( void * pv, magnetometer_sensor_t *magnetometer );
typedef bool (*readGyroFn)( void * pv, gyrometer_sensor_t *gyrometer );

/* passed to the senor in the 'init' call. If the hardware supports a feature it will fill in the appropriate callback */
typedef struct sensorCallback_st
{
	readTempFn 			readTemp;
	readAccelerometerFn readAccelerometer;
	readMagnetometerFn  readMagnetometer;
	readGyroFn 			readGyro;
} sensorCallback_st;

/* passed to the sensor in the init call. Used to pass along configuration preferences. */
typedef struct sensorConfig_st
{
	void *i2cCtx;
} sensorConfig_st;



#endif
