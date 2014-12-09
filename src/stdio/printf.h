#ifndef __PRINTF_H__
#define __PRINTF_H__

#include <stdarg.h>

signed int vsnprintf_aligned(char *pStr, size_t length, const char *pFormat, va_list ap);
signed int vsnprintf_unaligned(char *pStr, size_t length, const char *pFormat, va_list ap);

#endif /* __PRINTF_H__ */
