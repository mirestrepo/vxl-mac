#ifndef vcl_gcc_libstdcxx_v3_set_txx_
#define vcl_gcc_libstdcxx_v3_set_txx_

#include <vcl/vcl_set.h>
#include <vcl/vcl_compiler.h>



#define VCL_SET_INSTANTIATE(T, Comp)\
template class vcl_set<T, Comp >;

#endif
