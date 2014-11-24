#ifndef __NIZFLIGHT_STDIO_H__
#define __NIZFLIGHT_STDIO_H__

#define EOF -1

typedef struct FILE
{
	int dummy;
} FILE;

#define stdout (FILE *)0
#define stderr (FILE *)1

signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap);
signed int snprintf(char *pString, size_t length, const char *pFormat, ...);
signed int vsprintf(char *pString, const char *pFormat, va_list ap);
signed int vfprintf(FILE *pStream, const char *pFormat, va_list ap);
signed int vprintf(const char *pFormat, va_list ap);
signed int fprintf(FILE *pStream, const char *pFormat, ...);
signed int printf(const char *pFormat, ...);
signed int sprintf(char *pStr, const char *pFormat, ...);
signed int puts(const char *pStr);
signed int fputc(signed int c, FILE *pStream);
signed int fputs(const char *pStr, FILE *pStream);

#endif /* __NIZFLIGHT_STDIO_H__ */
