// -*- c++ -*-
#ifndef vcl_complex_txx_
#define vcl_complex_txx_

#include "vcl_complex.h"

#if !VCL_USE_NATIVE_COMPLEX
# include "emulation/vcl_complex.txx"
#elif defined(VCL_EGCS)
# include "egcs/vcl_complex.txx"
#elif defined(VCL_GCC_295) && !defined(GNU_LIBSTDCXX_V3)
# include "gcc-295/vcl_complex.txx"
#elif defined(VCL_SUNPRO_CC)
# include "sunpro/vcl_complex.txx"
#elif defined(VCL_SGI_CC)
# include "sgi/vcl_complex.txx"
#elif defined(VCL_WIN32)
# include "win32/vcl_complex.txx"
#else
# include "iso/vcl_complex.txx"
#endif

#endif
