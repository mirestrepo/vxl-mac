#ifndef vnl_matrix_fixed_h_
#define vnl_matrix_fixed_h_
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vnl/vnl_matrix_fixed.h

//:
// \file
// \brief fixed size matrix
//
// A subclass of vnl_matrix_fixed_ref,
// all storage is local and all vnl_matrix operations are valid.
//
// \author Andrew W. Fitzgibbon, Oxford RRG
// \date   04 Aug 96
// \verbatim
// Modifications:
// Peter Vanroose, 23 Nov 1996:  added explicit copy constructor
// LSB (Manchester) 15/03/2001:  added Binary I/O and tidied up the documentation
// \endverbatim
//-----------------------------------------------------------------------------

#include <vcl_cassert.h>
#include <vnl/vnl_matrix_fixed_ref.h>

//: Fixed size matrix
//  vnl_matrix_fixed<T,m,n> - Fixed size matrix.
//  A subclass of vnl_matrix_fixed_ref,
//  all storage is local and all vnl_matrix operations are valid.

template <class T, int m, int n>
class vnl_matrix_fixed : public vnl_matrix_fixed_ref<T,m,n> {
  T space[m*n]; // Local storage
public:

//: Construct an empty m*n matrix
  vnl_matrix_fixed() : vnl_matrix_fixed_ref<T,m,n>(space) {}

//: Construct an m*n matrix and fill with value
  vnl_matrix_fixed(const T& value):vnl_matrix_fixed_ref<T,m,n>(space) {
    int i = m*n;
    while (i--)
      space[i] = value;
  }

//: Construct an m*n Matrix and copy data into it row-wise.
  vnl_matrix_fixed(const T* datablck) : vnl_matrix_fixed_ref<T,m,n>(space) {
    vcl_memcpy(space, datablck, m*n*sizeof(T));
  }

//: Construct an m*n Matrix and copy rhs into it.   Abort if rhs is
// not the same size.
  vnl_matrix_fixed(const vnl_matrix<T>& rhs) : vnl_matrix_fixed_ref<T,m,n>(space) {
    assert(rhs.rows() == m && rhs.columns() == n);
    vcl_memcpy(space, rhs.data_block(), m*n*sizeof(T));
  }

//  Destruct the m*n matrix.
// An explicit destructor seems to be necessary, at least for gcc 3.0.0,
// to avoid the compiler generating multiple versions of it.
// (This way, a weak symbol is generated; otherwise not.  A bug of gcc 3.0.)
  ~vnl_matrix_fixed() {}

//: Copy a vnl_matrix into this.   Abort if rhs is
// not the same size.
  vnl_matrix_fixed<T,m,n>& operator=(const vnl_matrix<T>& rhs) {
    assert(rhs.rows() == m && rhs.columns() == n);
    vcl_memcpy(space, rhs.data_block(), m*n*sizeof(T));
    return *this;
  }

//: Copy another vnl_matrix_fixed<T,m,n> into this.
  vnl_matrix_fixed<T,m,n>& operator=(const vnl_matrix_fixed<T, m, n>& rhs) {
    vcl_memcpy(space, rhs.data_block(), m*n*sizeof(T));
    return *this;
  }

  vnl_matrix_fixed(const vnl_matrix_fixed<T,m,n>& rhs) : vnl_matrix_fixed_ref<T,m,n>(space) {
    vcl_memcpy(space, rhs.data_block(), m*n*sizeof(T));
  }
};

#ifndef VCL_SUNPRO_CC

 template <class T, int M, int N, int O>
 //: Multiply two conformant vnl_matrix_fixed (M x N) times (N x O)
 vnl_matrix_fixed<T, M, O> operator*(const vnl_matrix_fixed<T, M, N>& a, const vnl_matrix_fixed<T, N, O>& b)
 {
  vnl_matrix_fixed<T, M, O> out;
  for(int i = 0; i < M; ++i)
    for(int j = 0; j < O; ++j) {
      T accum = a(i,0) * b(0,j);
      for(int k = 1; k < N; ++k)
        accum += a(i,k) * b(k,j);
      out(i,j) = accum;
    }
  return out;
 }
#endif

#define VNL_MATRIX_FIXED_PAIR_INSTANTIATE(T, M, N, O) \
extern "please include vnl/vnl_matrix_fixed.txx instead"

#endif // vnl_matrix_fixed_h_
