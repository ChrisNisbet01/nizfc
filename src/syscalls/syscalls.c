#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

extern int  _end;
extern int  __StackLimit;

#define RAM_END (unsigned char *)&__StackLimit

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

static unsigned char *heap = (unsigned char *)&_end;
extern caddr_t _sbrk(int nbytes)
{
  if (heap + nbytes < RAM_END-4096) {
    unsigned char *prev_heap = heap;
    heap += nbytes;
    return (caddr_t) prev_heap;
  }
  else {
    errno = ENOMEM;
    return ((void*)-1);
  }
}

int _getpid(void)
{
	return 1;
}


int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}


void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}


int _write(int file, char *ptr, int len)
{
	return -1;
}


int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}


int _read(int file, char *ptr, int len)
{
	return 0;
}


int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}


int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}


int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}


int _times(struct tms *buf)
{
	return -1;
}


int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}


int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}


int _fork(void)
{
	errno = EAGAIN;
	return -1;
}


int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

