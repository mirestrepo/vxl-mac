#ifndef vcl_ios_h_
#define vcl_ios_h_
#ifdef __GNUC__
#pragma interface
#endif
/*
  fsm@robots.ox.ac.uk
*/

#include "vcl_compiler.h"

// FIXME
//#include "vcl_iostream.h"

#if defined(VCL_SGI_CC_720)
# include "sgi/vcl_ios.h"

#elif defined(VCL_GCC) && !VCL_CXX_HAS_HEADER_IOS
# include "gcc-295/vcl_ios.h"

#elif !VCL_CXX_HAS_HEADER_IOS
# include "vcl_iostream.h" // should do it
# define vcl_generic_ios_STD /* */
# include "generic/vcl_ios.h"

#else // -------------------- ISO
# include "iso/vcl_ios.h"
#endif

#endif // vcl_ios_h_
