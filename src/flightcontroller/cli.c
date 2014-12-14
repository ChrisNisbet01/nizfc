#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <printf.h>
#include <stdarg.h>
#include <ctype.h>
#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <serial.h>
#include <serial_msp.h>

#define CLI_LINEBUFFER_SIZE 	64
#define MAX_COMMAND_LINE_ARGS	6
#define CLI_OUTPUT_BUFFER_SIZE	256	/* buffer used to output CLI responses */

static const char cliPrompt[] = "#>";
static const char cliOK[]     = "OK";
static const char cliERROR[]     = "ERROR";
static const char pError[] = "increase CLI_OUTPUT_BUFFER_SIZE";

typedef enum special_char_t
{
	special_char_none,
	special_char_escape,
	special_char_arrow
} special_char_t;

typedef struct cliCtx_st
{
	serial_port_st  * serialPort;							/* used to indicate the port to write to */
	char 			lineBuffer[CLI_LINEBUFFER_SIZE+1];
	uint8_t 		linebuffer_cursor_position;
	char			*commandArgs[MAX_COMMAND_LINE_ARGS];	/* will point into lineBuffer */

	char			outputBuffer[CLI_OUTPUT_BUFFER_SIZE];
	uint16_t		outputBufferLen;
	mspContext_st	mspContext;
	bool 			receivingMSP;
} cliCtx_st;

static cliCtx_st cli[2];

static int clifputc(void *pv, signed int c)
{
	cliCtx_st *pctx	= pv;

	return pctx->serialPort->methods->writeCharBlockingWithTimeout( pctx->serialPort->serialCtx, c, 50 );
}

static int clifputs(void *pv, const char *pStr)
{
	cliCtx_st *pctx	= pv;
    signed int num = 0;

	if ( pctx->serialPort->methods->writeBulkBlockingWithTimeout != NULL )
	{
		return pctx->serialPort->methods->writeBulkBlockingWithTimeout( pctx->serialPort->serialCtx, (uint8_t const *)pStr, strlen( pStr ), 50 );
	}
	else
	{
	    while (*pStr != 0)
	   	{
	        if (clifputc(pctx, *pStr) == -1)
	            return -1;
	        num++;
	        pStr++;
	    }
	}

    return num;
}

static void cliFlushBufferedOutput( cliCtx_st *pctx )
{
	if ( pctx->serialPort->methods->writeBulkBlockingWithTimeout != NULL && pctx->outputBufferLen > 0 )
	{
		pctx->serialPort->methods->writeBulkBlockingWithTimeout( pctx->serialPort->serialCtx, (uint8_t const *)pctx->outputBuffer, pctx->outputBufferLen, 50 );
		pctx->outputBufferLen = 0;
	}
}

static void cliPutBufferedChar( void *pv, uint8_t ch )
{
	cliCtx_st *pctx	= pv;

	if ( pctx->serialPort->methods->writeBulkBlockingWithTimeout == NULL )
	{
		/* write out each char */
		pctx->serialPort->methods->writeCharBlockingWithTimeout( pctx->serialPort->serialCtx, ch, 50 );
	}
	else
	{
		if ( pctx->outputBufferLen < sizeof(pctx->outputBuffer) )
		{
			pctx->outputBuffer[pctx->outputBufferLen++] = ch;
		}
		if ( pctx->outputBufferLen == sizeof(pctx->outputBuffer) )
		{
			cliFlushBufferedOutput( pctx );
		}
	}
}

static int clivfprintf(void *pv, const char *pFormat, va_list ap)
{
	cliCtx_st *pctx	= pv;

    /* Write formatted string in buffer */
    if ((unsigned)vsnprintf_aligned(pctx->outputBuffer, sizeof(pctx->outputBuffer), pFormat, ap) >= sizeof(pctx->outputBuffer)) {

        clifputs(pctx, pError);
        return -1;
    }

    /* Display string */
    return clifputs(pctx, pctx->outputBuffer);
}

int cliPrintf(void *pv, const char *pFormat, ...)
{
	cliCtx_st *pctx	= pv;
    va_list ap;
    signed int result;

    /* Forward call to clivfprintf */
    va_start(ap, pFormat);
    result = clivfprintf(pctx, pFormat, ap);
    va_end(ap);

    return result;
}


static bool cliGetCommand( cliCtx_st *pctx, char const ch )
{
	bool got_command = false;
	char ch7bit = ch & 0x7f;

	if( ch7bit == '\n' ) /* just echo linefeeds */
	{
	   cliPrintf(pctx, "\n");
	}
	else if ( ch7bit == '\b' )    /* Backspace */
	{
		if ( pctx->linebuffer_cursor_position > 0 )
		{
			cliPrintf(pctx, "\b \b");
			pctx->linebuffer_cursor_position--;
		}
	}
	else if ( ch7bit != '\r' )
	{
		if ( pctx->linebuffer_cursor_position < CLI_LINEBUFFER_SIZE )     /* Not exceeded max input length */
        {
        	pctx->lineBuffer[pctx->linebuffer_cursor_position++] = ch7bit;
        	/* echo the character */
        	cliPrintf(pctx, "%c", ch7bit);
	  	}
	}
	else
	{
		/* got carriage return */
		pctx->lineBuffer[pctx->linebuffer_cursor_position] = '\0';
       	cliPrintf(pctx, "\r");	/* echo the CR */

		got_command = true;
	}

	return got_command;
}

/*----------------------------------------------------------------------------*/
static int commandLineParse( char * const linebuffer, int max_args, char **argv )
/*----------------------------------------------------------------------------*/
{
	char * line = linebuffer;
	int argc;
	/* split a command line into command and args. */

	for ( argc = 0; argc < max_args && *line != '\0'; )
	{
		/* Skip leading white space */
		while( *line != '\0' && isspace( (int)*line ) )
			line++;
		if(*line == '\0')
		   break;
		argv[argc++] = line;	/* Beginning of token */
		while( *line != '\0' && !isspace( (int)*line ) )
			line++;
		if( *line != '\0' )	/* NUL terminate each arg */
			*line++ = '\0';
	}

	if( argc < max_args )
		argv[argc] = NULL;

	/* indicate the number of args found */
	return argc;
}


static void cliPrintPrompt( cliCtx_st *pctx )
{
	cliPrintf(pctx, "\n%s", cliPrompt);
}

void cliHandleNewChar( void *pv, char const ch )
{
	cliCtx_st *pctx = pv;

	if ( pctx->receivingMSP == true )
	{
		if ( mspProcess( &pctx->mspContext, ch ) == false )
			pctx->receivingMSP = false;
		cliFlushBufferedOutput( pctx );
	}
	else if ( pctx->linebuffer_cursor_position == 0 && mspProcess( &pctx->mspContext, ch ) == true )
	{
		pctx->receivingMSP = true;
		cliFlushBufferedOutput( pctx );
	}
	else if ( cliGetCommand( pctx, ch ) == true )
	{
		int argc;
		/*
			We now have a command sitting in the command buffer.
			Split it up into arguments.
		*/
		if ( (argc=commandLineParse( pctx->lineBuffer, MAX_COMMAND_LINE_ARGS, pctx->commandArgs )) > 0 )
		{
			int command_result;
			/* run the command */
			command_result = runCommand( argc, pctx->commandArgs, pctx );
			switch (command_result)
			{
				case poll_result_ok:
					cliPrintf( pctx, "\n%s", cliOK );
					break;
				default:
					cliPrintf( pctx, "\n%s", cliERROR );
					break;
			}
		}

		/* clear the previous command */
		pctx->linebuffer_cursor_position = 0;

		/* output the prompt */
		cliPrintPrompt( pctx );
	}
}

void *initCli( serial_port_st * serialPort )
{
	unsigned int cli_index;

	for (cli_index = 0; cli_index < ARRAY_SIZE(cli); cli_index++ )
	{
		cliCtx_st *pctx = &cli[cli_index];

		if ( pctx->serialPort == NULL )
		{
			pctx->linebuffer_cursor_position = 0;
			pctx->serialPort = serialPort;
			pctx->receivingMSP = false;
			initMSPContext( &pctx->mspContext, pctx, (uint8_t *)pctx->lineBuffer, sizeof(pctx->lineBuffer),	cliPutBufferedChar);

			return pctx;
		}
	}

	return NULL;
}
