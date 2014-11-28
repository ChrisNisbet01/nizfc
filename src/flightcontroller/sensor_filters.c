#include <stdint.h>
#include <filter.h>
#include <sensor_configuration.h>

void filterAccValues( float * newValues, float * filteredValues )
{
	filterValues( filteredValues, filteredValues, newValues, acc_configuration[0].lpf_factor, 3 );
}

void filterGyroValues( float * newValues, float * filteredValues )
{
	filterValues( filteredValues, filteredValues, newValues, gyro_configuration[0].lpf_factor, 3 );
}

void filterMagValues( float * newValues, float * filteredValues )
{
	filterValues( filteredValues, filteredValues, newValues, mag_configuration[0].lpf_factor, 3 );
}


