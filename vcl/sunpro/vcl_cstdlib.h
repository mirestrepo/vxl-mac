#ifndef vcl_sunpro_cstdlib_h_
#define vcl_sunpro_cstdlib_h_
// .NAME vcl_cstdlib
// .INCLUDE vcl_cstdlib.h
// .FILE vcl_cstdlib.cxx

#include "../iso/vcl_cstdlib.h"

#undef  vcl_abs
#define vcl_abs vcl_abs
inline int  vcl_abs(int  x) { return x >= 0    ? x : -x; }
inline long vcl_abs(long x) { return x >= 0L   ? x : -x; }

#endif // vcl_cstdlib_h_
