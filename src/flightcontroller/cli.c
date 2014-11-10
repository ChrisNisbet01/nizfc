#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <utils.h>
#include <cmds.h>

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
	int 			(*putChar)( int ch );					/* function to call to put a character to the attached CLI terminal */
	char 			lineBuffer[CLI_LINEBUFFER_SIZE+1];
	uint8_t 		linebuffer_cursor_position;
	char			*commandArgs[MAX_COMMAND_LINE_ARGS];	/* will point into lineBuffer */

	char			outputBuffer[CLI_OUTPUT_BUFFER_SIZE];

} cliCtx_st;

static cliCtx_st cli[1];

static int clifputc(void *pv, signed int c)
{
	cliCtx_st *pctx	= pv;

	if (pctx->putChar != NULL)
		return pctx->putChar( c );
    return -1;
}

static int clifputs(void *pv, const char *pStr)
{
	cliCtx_st *pctx	= pv;
    signed int num = 0;

    while (*pStr != 0)
   	{
        if (clifputc(pctx, *pStr) == -1)
            return -1;
        num++;
        pStr++;
    }

    return num;
}

static int clivfprintf(void *pv, const char *pFormat, va_list ap)
{
	cliCtx_st *pctx	= pv;

    /* Write formatted string in buffer */
    if ((unsigned)vsnprintf(pctx->outputBuffer, sizeof(pctx->outputBuffer), pFormat, ap) >= sizeof(pctx->outputBuffer)) {

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
	else if ( ch7bit != '\r' && ch != ','/* temp debug */ )    /* not return */
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

	if ( cliGetCommand( pctx, ch ) == true )
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

void *initCli( int (*putChar)(int ch) )
{
	cliCtx_st *pctx = &cli[0];

	pctx->linebuffer_cursor_position = 0;
	pctx->putChar = putChar;

	return pctx;
}
