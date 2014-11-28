#ifndef __SENSOR_FILTERS_H__
#define __SENSOR_FILTERS_H__

void filterAccValues( float * newValues, float * filteredValues );
void filterGyroValues( float * newValues, float * filteredValues );
void filterMagValues( float * newValues, float * filteredValues );

#endif /* __SENSOR_FILTERS_H__ */
