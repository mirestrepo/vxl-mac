/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vnl_trace.h"

#include <vnl/vnl_matrix.h>

template <class T>
T vnl_trace(vnl_matrix<T> const &M) {
  T sum(0);
  unsigned N = (M.rows() < M.cols() ? M.rows() : M.cols());
  for (unsigned i=0; i<N; ++i)
    sum += M(i, i);
  return sum;
}

#define VNL_TRACE_INSTANTIATE(T) \
template T vnl_trace(vnl_matrix<T > const &);

VNL_TRACE_INSTANTIATE(float);
VNL_TRACE_INSTANTIATE(double);
