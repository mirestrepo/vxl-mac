#ifndef vcl_borland_cstdlib_h_
#define vcl_borland_cstdlib_h_
// This is a first attempt at a <cstdlib> for the Borland compiler - PVr,Dec.2003.
#include <cstdlib>

// If we define vcl_abs, for example, to ::abs, we have conflicts
// with std::abs(std::complex<T>) which *is* declared in the
// std namespace. To avoid these issues, we inject the math
// functions into the std namespace.

namespace std {
  //inline int abs(int x) { return x >= 0 ? x : -x; }
  inline long abs(long x) { return x >= 0 ? x : -x; }
  inline long labs(long x) { return x >= 0 ? x : -x; }
}

#ifndef vcl_abs
#define vcl_abs std::abs
#endif
#ifndef vcl_labs
#define vcl_labs std::labs
#endif
#ifndef vcl_div
#define vcl_div std::div
#endif
#ifndef vcl_ldiv
#define vcl_ldiv std::ldiv
#endif

#define vcl_generic_cstdlib_STD

#include "../generic/vcl_cstdlib.h"

#endif // vcl_borland_cstdlib_h_
