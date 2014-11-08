#include <ctype.h>

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

