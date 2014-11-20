#ifndef __CLI_H__
#define __CLI_H__

#include <serial.h>

void cliHandleNewChar( void *pv, char const ch );
void *initCli( serial_port_st * uart );
int cliPrintf(void * pv, const char *pFormat, ...);

#endif
