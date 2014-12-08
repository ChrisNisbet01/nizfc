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
#define MEMORY_BARRIER()			asm volatile("": : :"memory")
int strtoint (char const * str, unsigned int  * pint );
float limitFloat( float value, float lowLimit, float highLimit);
int limit( int value, int lowLimit, int highLimit);
float scale(int32_t value, int32_t srcMin, int32_t srcMax, float destMin, float destMax);
float normaliseHeading( float heading );

void delayMilliseconds( unsigned int milliseconds );



#endif /*  __UTILS_H__ */
