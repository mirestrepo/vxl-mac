#ifndef vcl_stack_h_
#define vcl_stack_h_

#include <vcl/vcl_compiler.h>

#if !VCL_USE_NATIVE_STL
# include <vcl/emulation/vcl_stack.h>

#elif defined(__GNUC__)
# include <stack.h>
# define vcl_stack stack

#else
# include <vcl/iso/vcl_stack.h>
#endif

#endif
