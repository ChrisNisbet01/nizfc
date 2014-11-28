#ifndef __SENSOR_CONFIGURATION_H__
#define __SENSOR_CONFIGURATION_H__

#define NB_ACC_CONFIGURATIONS	1
#define NB_GYRO_CONFIGURATIONS	1
#define NB_MAG_CONFIGURATIONS	1

typedef struct sensor_configuration_st
{
	uint16_t lpf_factor;
} sensor_configuration_st;


extern sensor_configuration_st acc_configuration[NB_ACC_CONFIGURATIONS];
extern sensor_configuration_st gyro_configuration[NB_GYRO_CONFIGURATIONS];
extern sensor_configuration_st mag_configuration[NB_MAG_CONFIGURATIONS];

#endif /* __SENSOR_CONFIGURATION_H__ */

