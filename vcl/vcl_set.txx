#ifndef vcl_set_txx_
#define vcl_set_txx_
// -*- c++ -*-

#include "vcl_set.h"

#if VCL_USE_IMPLICIT_TEMPLATES
# include "iso/vcl_set.txx"
#elif !VCL_USE_NATIVE_STL
# include "emulation/vcl_set.txx"
#elif defined(VCL_EGCS)
# include "egcs/vcl_set.txx"
#elif defined(VCL_GCC_295) && !defined(GNU_LIBSTDCXX_V3)
# include "gcc-295/vcl_set.txx"
#elif defined(GNU_LIBSTDCXX_V3)
# include "gcc-libstdcxx-v3/vcl_set.txx"
#elif defined(VCL_SUNPRO_CC)
# include "sunpro/vcl_set.txx"
#elif defined(VCL_SGI_CC)
# include "sgi/vcl_set.txx"
#elif defined(VCL_WIN32)
# include "win32/vcl_set.txx"
#else
# include "iso/vcl_set.txx"
#endif

#endif
