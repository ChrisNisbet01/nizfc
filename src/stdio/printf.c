/**************************************************************************//*****
 * @file     printf.c
 * @brief    Implementation of several stdio.h methods, such as printf(),
 *           sprintf() and so on. This reduces the memory footprint of the
 *           binary when using those methods, compared to the libc implementation.
 ********************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <serial.h>
//#include <vsnprintf.h>

/**
 * @brief  Transmit a char, if you want to use printf(),
 *         you need implement this function
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.
 */
extern serial_port_st *debug_port;

/** Maximum string size allowed (in bytes). */
#define MAX_STRING_SIZE         256

static void PrintChar(char c)
{
	/* Send a char like:
	   while(Transfer not completed);
	   Transmit a char;
	*/
	if (debug_port != NULL)	// TODO: confgurable
	{
		debug_port->methods->writeCharBlockingWithTimeout( debug_port->serialCtx, (uint8_t)c, 2 );
	}
}

/**
 * @brief  Writes a character inside the given string. Returns 1.
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.
 */
static signed int PutChar(char *pStr, char c)
{
    *pStr = c;
    return 1;
}


/**
 * @brief  Writes a string inside the given string.
 *
 * @param  pStr     Storage string.
 * @param  pSource  Source string.
 * @return  The size of the written
 */
static signed int PutString(char *pStr, char fill, signed int width, const char *pSource)
{
    signed int num = 0;

	if ( pSource == NULL )
		pSource = "(NULL)";

	if (width > (signed)strlen(pSource))
	{
		int i = width - strlen(pSource);
		while( i-- )
		{
	        *pStr++ = fill;
	        num++;
		}
	}

    while (*pSource != 0) {

        *pStr++ = *pSource++;
        num++;
    }

    return num;
}


/**
 * @brief  Writes an unsigned int inside the given string, using the provided fill &
 *         width parameters.
 *
 * @param  pStr  Storage string.
 * @param  fill  Fill character.
 * @param  width  Minimum integer width.
 * @param  value  Integer value.
 */
static signed int PutUnsignedInt(
    char *pStr,
    char fill,
    signed int width,
    unsigned int value)
{
    signed int num = 0;

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((value / 10) > 0) {

        num = PutUnsignedInt(pStr, fill, width, value / 10);
        pStr += num;
    }

    /* Write filler characters */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (value % 10) + '0');

    return num;
}


/**
 * @brief  Writes a signed int inside the given string, using the provided fill & width
 *         parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param value  Signed integer value.
 */
static signed int PutSignedInt(
    char *pStr,
    char fill,
    signed int width,
    signed int value)
{
    signed int num = 0;
    unsigned int absolute;

    /* Compute absolute value */
    if (value < 0) {

        absolute = -value;
    }
    else {

        absolute = value;
    }

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((absolute / 10) > 0) {

        if (value < 0) {

            num = PutSignedInt(pStr, fill, width, -(absolute / 10));
        }
        else {

            num = PutSignedInt(pStr, fill, width, absolute / 10);
        }
        pStr += num;
    }
    else {

        /* Reserve space for sign */
        if (value < 0) {

            width--;
        }

        /* Write filler characters */
        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }

        /* Write sign */
        if (value < 0) {

            num += PutChar(pStr, '-');
            pStr++;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (absolute % 10) + '0');

    return num;
}


/**
 * @brief  Writes an hexadecimal value into a string, using the given fill, width &
 *         capital parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param maj    Indicates if the letters must be printed in lower- or upper-case.
 * @param value  Hexadecimal value.
 *
 * @return  The number of char written
 */
static signed int PutHexa(
    char *pStr,
    char fill,
    signed int width,
    unsigned char maj,
    unsigned int value)
{
    signed int num = 0;

    /* Decrement width */
    width--;

    /* Recursively output upper digits */
    if ((value >> 4) > 0) {

        num += PutHexa(pStr, fill, width, maj, value >> 4);
        pStr += num;
    }
    /* Write filler chars */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write current digit */
    if ((value & 0xF) < 10) {

        PutChar(pStr, (value & 0xF) + '0');
    }
    else if (maj) {

        PutChar(pStr, (value & 0xF) - 10 + 'A');
    }
    else {

        PutChar(pStr, (value & 0xF) - 10 + 'a');
    }
    num++;

    return num;
}

static char *_i2a(unsigned i, char *a, unsigned base)
{
    if (i / base > 0)
        a = _i2a(i / base, a, base);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % base];
    return a + 1;
}

static char *itoa(int i, char *a, int base)
{
    if ((base < 2) || (base > 36))
        base = 10;
    if (i < 0) {
        *a = '-';
        *_i2a(-(unsigned) i, a + 1, base) = 0;
    } else
        *_i2a(i, a, base) = 0;
    return a;
}

static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t)(x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static signed int PutFloat( char *pStr, float value )
{
	ftoa( value, pStr );
	return strlen( pStr );
}

/* Global Functions ----------------------------------------------------------- */


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ap      Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
    char          fill;
    unsigned char width;
    signed int    num = 0;
    size_t    size = 0;

    /* Clear the string */
    if (pStr) {

        *pStr = 0;
    }

    /* Phase string */
    while (*pFormat != 0 && size < length) {

        /* Normal character */
        if (*pFormat != '%') {

            *pStr++ = *pFormat++;
            size++;
        }
        /* Escaped '%' */
        else if (*(pFormat+1) == '%') {

            *pStr++ = '%';
            pFormat += 2;
            size++;
        }
        /* Token delimiter */
        else {

            fill = ' ';
            width = 0;
            pFormat++;

            /* Parse filler */
            if (*pFormat == '0') {

                fill = '0';
                pFormat++;
            }

            /* Parse width */
            while ((*pFormat >= '0') && (*pFormat <= '9')) {

                width = (width*10) + *pFormat-'0';
                pFormat++;
            }

            /* Check if there is enough space */
            if (size + width > length) {

                width = length - size;
            }

            /* Parse type */
            switch (*pFormat) {
            case 'd':
            case 'i': num = PutSignedInt(pStr, fill, width, va_arg(ap, signed int)); break;
            case 'u': num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int)); break;
            case 'p':
            case 'x': num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int)); break;
            case 'X': num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int)); break;
            case 's': num = PutString(pStr, fill, width, va_arg(ap, char *)); break;
            case 'c': num = PutChar(pStr, va_arg(ap, unsigned int)); break;
            case 'f': num = PutFloat(pStr, va_arg(ap, double)); break;
            case 'g':
            {
            	float *pf = (float *)va_arg(ap, long);
            	num = PutFloat(pStr, *pf);
            	break;
            }
            default:
                return EOF;
            }

            pFormat++;
            pStr += num;
            size += num;
        }
    }

    /* NULL-terminated (final \0 is not counted) */
    if (size < length) {

        *pStr = 0;
    }
    else {

        *(--pStr) = 0;
        size--;
    }

    return size;
}

/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ...     Other arguments
 *
 * @return  The number of characters written.
 */
signed int snprintf(char *pString, size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);
    return rc;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pString  Destination string.
 * @param length   Length of Destination string.
 * @param pFormat  Format string.
 * @param ap       Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsprintf(char *pString, const char *pFormat, va_list ap)
{
   return vsnprintf(pString, MAX_STRING_SIZE, pFormat, ap);
}

/**
 * @brief  Outputs a formatted string on the given stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string
 * @param ap       Argument list.
 */
signed int vfprintf(FILE *pStream, const char *pFormat, va_list ap)
{
    char pStr[MAX_STRING_SIZE];
    static const char pError[] = "printf.c: increase MAX_STRING_SIZE\n\r";

    /* Write formatted string in buffer */
    if (vsprintf(pStr, pFormat, ap) >= MAX_STRING_SIZE) {

        fputs(pError, stderr);
    }

    /* Display string */
    return fputs(pStr, pStream);
}

/**
 * @brief  Outputs a formatted string on the given stream, using a variable
 *         number of arguments.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string.
 */
signed int fprintf(FILE *pStream, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    /* Forward call to vfprintf */
    va_start(ap, pFormat);
    result = vfprintf(pStream, pFormat, ap);
    va_end(ap);

    return result;
}

/**
 * @brief  Implementation of fputc using the DBGU as the standard output. Required
 *         for printf().
 *
 * @param c        Character to write.
 * @param pStream  Output stream.
 * @param The character written if successful, or -1 if the output stream is
 *        not stdout or stderr.
 */
signed int fputc(signed int c, FILE *pStream)
{
    if ((pStream == stdout) || (pStream == stderr)) {

    	PrintChar(c);

        return c;
    }
    else {

        return EOF;
    }
}


/**
 * @brief  Implementation of fputs using the DBGU as the standard output. Required
 *         for printf().
 *
 * @param pStr     String to write.
 * @param pStream  Output stream.
 *
 * @return  Number of characters written if successful, or -1 if the output
 *          stream is not stdout or stderr.
 */
signed int fputs(const char *pStr, FILE *pStream)
{
    signed int num = 0;

	if (debug_port != NULL && debug_port->methods->writeBulkBlockingWithTimeout != NULL)	// TODO: configurable
	{
		debug_port->methods->writeBulkBlockingWithTimeout( debug_port->serialCtx, (uint8_t const *)pStr, strlen( pStr ), 50 );
	}
	else
	{
	    while (*pStr != 0)
	    {
	        if (fputc(*pStr, pStream) == -1)
	        {
	            return -1;
	        }
	        num++;
	        pStr++;
	    }
	}

    return num;
}
/**
 * @brief  Outputs a formatted string on the DBGU stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pFormat  Format string.
 * @param ap  Argument list.
 */
signed int vprintf(const char *pFormat, va_list ap)
{
    return vfprintf(stdout, pFormat, ap);
}

/**
 * @brief  Outputs a formatted string on the DBGU stream, using a variable number of
 *         arguments.
 *
 * @param  pFormat  Format string.
 */
signed int printf(const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    /* Forward call to vprintf */
    va_start(ap, pFormat);
    result = vprintf(pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Writes a formatted string inside another string.
 *
 * @param pStr     torage string.
 * @param pFormat  Format string.
 */
signed int sprintf(char *pStr, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to vsprintf
    va_start(ap, pFormat);
    result = vsprintf(pStr, pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Outputs a string on stdout.
 *
 * @param pStr  String to output.
 */
signed int puts(const char *pStr)
{
    return fputs(pStr, stdout);
}



