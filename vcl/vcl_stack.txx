#ifndef vcl_stack_txx_
#define vcl_stack_txx_

#include <vcl/vcl_stack.h>

#undef VCL_STACK_INSTANTIATE

#if !VCL_USE_NATIVE_STL
# include <vcl/emulation/vcl_stack.txx>
#elif defined(VCL_EGCS)
# include <vcl/egcs/vcl_stack.txx>
#elif defined(VCL_GCC_295) && !defined(GNU_LIBSTDCXX_V3)
# include <vcl/gcc-295/vcl_stack.txx>
#elif defined(VCL_GCC_295) && defined(GNU_LIBSTDCXX_V3)
# include <vcl/gcc-libstdcxx-v3/vcl_stack.txx>
#elif defined(VCL_SUNPRO_CC)
# include <vcl/sunpro/vcl_stack.txx>
#elif defined(VCL_SGI_CC)
# include <vcl/sgi/vcl_stack.txx>
#elif defined(VCL_WIN32)
# include <vcl/win32/vcl_stack.txx>
#else
# include <vcl/iso/vcl_stack.txx>
#endif

#endif
