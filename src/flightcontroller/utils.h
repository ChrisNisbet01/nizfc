#ifndef __UTILS_H__
#define __UTILS_H__

#define UNUSED(x) (void)(x)

#define max(a,b) \
({ typeof (a) _a = (a); \
   typeof (b) _b = (b); \
 _a > _b ? _a : _b; })

#define min(a,b) \
({ typeof (a) _a = (a); \
   typeof (b) _b = (b); \
 _a < _b ? _a : _b; })

#define COMPILE_TIME_ASSERT(expr)  {char uname[(expr)?1:-1];(void)uname;}
#define ARRAY_SIZE(array)			(sizeof(array)/sizeof(array[0]))

int strtoint (char const * str, unsigned int  * pint );

#endif /*  __UTILS_H__ */
