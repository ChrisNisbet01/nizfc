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


#endif /*  __UTILS_H__ */
