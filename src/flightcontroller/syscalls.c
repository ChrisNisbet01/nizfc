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

