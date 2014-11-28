#ifndef __FILTER_H__
#define __FILTER_H__

float filterValue( float const oldValue, float const newValue, uint32_t const factor );
void filterValues( float *results, float const * const oldValues, float const * const newValues, uint32_t const factor, uint_fast8_t const nbValues );


#endif /* __FILTER_H__ */
