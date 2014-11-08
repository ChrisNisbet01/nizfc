#ifndef __CLI_H__
#define __CLI_H__

void cliHandleNewChar( void *pv, char const ch );
void *initCli( int (*putChar)(int ch) );
int cliPrintf(void *pv, const char *pFormat, ...);

#endif
