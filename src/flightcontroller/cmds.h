#ifndef __CMDS_H__
#define __CMDS_H__

typedef struct command_st
{
	char const * name;		   /* Name of command */
	int (* handler)(int argc, char *argv[], void *p);
} command_st;

int runCommand( int argc, char **argv, void *pv );
int runCommandHandler( command_st const * const commands, uint32_t nb_commands, void *pv );

#endif /* __CMDS_H__ */
