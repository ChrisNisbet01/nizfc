#include <stdint.h>

float filterValue( float const oldValue, float const newValue, uint32_t const factor )
{
	return ((oldValue * factor) + newValue)/(1.0f+factor);
}

void filterValues( float *results, float const * const oldValues, float const * const newValues, uint32_t const factor, uint_fast8_t const nbValues )
{
	uint_fast8_t index;
	float divisor;

	divisor = 1.0f + factor;

	for ( index = 0; index < nbValues; index++ )
	{
		results[index] = ((oldValues[index] * factor) + newValues[index])/divisor;
	}
}

