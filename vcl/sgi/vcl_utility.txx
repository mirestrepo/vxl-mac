#ifndef vcl_win32_utility_txx_
#define vcl_win32_utility_txx_
/*
  fsm@robots.ox.ac.uk
*/

#include <vcl/vcl_utility.h>

#define VCL_PAIR_INSTANTIATE(T1, T2) \
template struct vcl_pair<T1, T2 >;

#endif
