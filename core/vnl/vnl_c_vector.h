#ifndef vnl_c_vector_h_
#define vnl_c_vector_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	vnl_c_vector - Math on blocks of memory
// .LIBRARY	vnl
// .HEADER	vxl package
// .INCLUDE	vnl/vnl_c_vector.h
// .FILE	vnl_c_vector.txx
//
// .SECTION Description
//    vnl_c_vector interfaces to lowlevel memory-block operations.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 12 Feb 98
//
// .SECTION Modifications
//     980212 AWF Initial version.
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_numeric_traits.h>

// avoid messing about with aux_* functions for gcc 2.7 -- fsm
template <class T, class S> void vnl_c_vector_one_norm(T const *p, unsigned n, S *out);
template <class T, class S> void vnl_c_vector_two_norm(T const *p, unsigned n, S *out);
template <class T, class S> void vnl_c_vector_inf_norm(T const *p, unsigned n, S *out);
template <class T, class S> void vnl_c_vector_two_norm_squared(T const *p, unsigned n, S *out);
template <class T, class S> void vnl_c_vector_rms_norm(T const *p, unsigned n, S *out);

template <class T>
class vnl_c_vector {
public:
  typedef typename vnl_numeric_traits<T>::abs_t abs_t;

  static T sum(const T* v, unsigned n);
  static inline abs_t squared_magnitude(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_two_norm_squared(p, n, &val); return val; }
  static void normalize(T *, unsigned n);
  static void apply(T const *, unsigned, T (*f)(T), T* v_out);
  static void apply(T const *, unsigned, T (*f)(const T&), T* v_out);
  static void copy    (T const *x, T       *y, unsigned);            // y[i]  = x[i]
  static void scale   (T const *x, T       *y, unsigned, T const &); // y[i]  = a*x[i]
  static void add     (T const *x, T const *y, T *z, unsigned);      // z[i]  = x[i] + y[i];
  static void subtract(T const *x, T const *y, T *z, unsigned);      // z[i]  = x[i] - y[i]
  static void multiply(T const *x, T const *y, T *z, unsigned);      // z[i]  = x[i] * y[i]
  static void divide  (T const *x, T const *y, T *z, unsigned);      // z[i]  = x[i] / y[i]
  static void negate  (T const *x, T       *y, unsigned);            // y[i]  = -x[i]
  static void invert  (T const *x, T       *y, unsigned);            // y[i]  = 1/x[i]
  static void saxpy   (T const &a, T const *x, T *y, unsigned);      // y[i] += a*x[i]
  static void fill    (T *x, unsigned, T const &v);                  // x[i]  = v
  static void reverse (T *x, unsigned);
  static T dot_product  (T const *, T const *, unsigned);
  static T inner_product(T const *, T const *, unsigned); // conjugate second
  static void conjugate(T const *, T *, unsigned);

  static T max_value(T const *, unsigned);
  static T min_value(T const *, unsigned);
  static T mean(T const *p, unsigned n) { return sum(p,n)/T(n); }

  // one_norm : sum of abs values
  // two_norm : sqrt of sum of squared abs values
  // inf_norm : max of abs values
  // two_nrm2 : sum of squared abs values
  // rms_norm : sqrt of mean sum of squared abs values
  static inline abs_t one_norm(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_one_norm(p, n, &val); return val; }
  static inline abs_t two_norm(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_two_norm(p, n, &val); return val; }
  static inline abs_t inf_norm(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_inf_norm(p, n, &val); return val; }
  static inline abs_t two_nrm2(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_two_norm_squared(p, n, &val); return val; }
  static inline abs_t rms_norm(T const *p, unsigned n)
    { abs_t val; vnl_c_vector_rms_norm(p, n, &val); return val; }

  // Memory allocation
  static T** allocate_Tptr(int n);
  static T*  allocate_T(int n);
  static void deallocate(T**, int n_when_allocated);
  static void deallocate(T*, int n_when_allocated);
};

#endif // vnl_c_vector_h_
