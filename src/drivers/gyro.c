#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stm32f30x_gpio.h>
#include <spi_stm32f30x.h>
#include <sensors.h>
#include <l3gd20.h>
#include <utils.h>



float gyroCurrentValues[3];
typedef struct gyro_cfg_st
{
	float lpfFactor;	/* smoothing factor applied to raw readings */
} gyro_cfg_st;

static gyro_cfg_st defaultGyroConfig =
{
	.lpfFactor = 0.8
};


bool readGyro( void *pv )
{
}
