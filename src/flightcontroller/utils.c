#include <stdint.h>
#include <ctype.h>
#include <coos.h>

void delayMilliseconds( unsigned int milliseconds )
{
	CoTickDelay( (milliseconds*CFG_SYSTICK_FREQ)/1000);
}

/*---------------------------------------------------------*/
int strtoint (char const * str, unsigned int  * pint )
/*---------------------------------------------------------*/
{
	unsigned int         value  = 0;

	if ( *str == 0 )
		return 0;      // if no string prvided return NO

	while (*str != '\0')
	{
		if ( isdigit ((int)*str) )
		{
			value *= 10;
			value += *str - '0';
		}
		else
			return 0;

		str++;
	}

	*pint = value;

	return 1;
}


float limitFloat( float value, float lowLimit, float highLimit)
{
    if (value < lowLimit)
        value = lowLimit;
    else if (value > highLimit)
        value = highLimit;

    return value;
}


int limit( int value, int lowLimit, int highLimit)
{
    if (value < lowLimit)
        value = lowLimit;
    else if (value > highLimit)
        value = highLimit;

    return value;
}

float scale(int32_t value, int32_t srcMin, int32_t srcMax, float destMin, float destMax)
{
	value = limit( value, srcMin, srcMax );

	return destMin + ((destMax - destMin)*(float)(value-srcMin))/(float)(srcMax-srcMin);
}

