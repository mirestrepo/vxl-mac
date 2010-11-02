// This is core/vil/vil_na.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// This file is a cut-and-paste of vil_na

#include "vil_na.h"
#include <vxl_config.h>
#include <vcl_istream.h>
#include <vcl_ios.h>

//: A particular qNaN to indicate not available.
// This returns the bit pattern 0x7ff00000000007a2, as used by Octave and R
// Don't assume that any VXL functions will treat the value as NA rather than NaN, unless
// explicitly documented.
double vil_na()
{
  double a;

#if VXL_HAS_INT_64
  *reinterpret_cast<vxl_uint_64*>(&a) = 0x7ff00000000007a2LL;
#else
# if VXL_BIG_ENDIAN
#  define hw 0
#  define lw 1
# else  // VXL_LITTLE_ENDIAN
#  define hw 1
#  define lw 0
# endif
  reinterpret_cast<vxl_uint_32*>(&a)[hw]=0x7ff00000;
  reinterpret_cast<vxl_uint_32*>(&a)[lw]=0x000007a2;
#endif

  return a;
}

//: True if parameter is specific NA qNaN.
// Tests for bit pattern 0x7ff00000000007a2, as used by Octave and R
bool vil_na_isna(double x)
{
#if VXL_HAS_INT_64
  return ((*reinterpret_cast<vxl_uint_64*>(&x))&0xfff7ffffffffffffLL)
    == 0x7ff00000000007a2LL;
#else
  return ((reinterpret_cast<vxl_int_32*>(&x)[hw]) & 0xffffffff) == 0x7ff00000 &&
         reinterpret_cast<vxl_int_32*>(&x)[lw] == 0x000007a2;
#endif
}


//----------------------------------------------------------------------
