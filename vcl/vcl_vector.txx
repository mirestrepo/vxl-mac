// -*- c++ -*-
#ifndef vcl_vector_txx_
#define vcl_vector_txx_

#include "vcl_vector.h"

#undef VCL_VECTOR_INSTANTIATE

#if !VCL_USE_NATIVE_STL
# include "emulation/vcl_vector.txx"
#elif defined(VCL_EGCS)
# include "egcs/vcl_vector.txx"
#elif defined(VCL_GCC_295) && !defined(GNU_LIBSTDCXX_V3)
# include "gcc-295/vcl_vector.txx"
#elif defined(GNU_LIBSTDCXX_V3)
# include "gcc-libstdcxx-v3/vcl_vector.txx"
#elif defined(VCL_SUNPRO_CC)
# include "sunpro/vcl_vector.txx"
#elif defined(VCL_SGI_CC)
# include "sgi/vcl_vector.txx"
#elif defined(VCL_WIN32)
# include "win32/vcl_vector.txx"
#else
# include "iso/vcl_vector.txx"
#endif

#endif
