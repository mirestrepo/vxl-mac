// This is vxl/vnl/algo/vnl_fft_base.h
#ifndef vnl_fft_base_h_
#define vnl_fft_base_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief In-place n-D fast fourier transform
// \author fsm@robots.ox.ac.uk

#include <vcl_complex.h>
#include <vnl/algo/vnl_fft_prime_factors.h>

//: Base class for in-place ND fast fourier transform.

export template <int D, class T>
struct vnl_fft_base
{
  vnl_fft_base() { }

  //: dir = +1/-1 according to direction of transform.
  void transform(vcl_complex<T> *signal, int dir);

 protected:
  //: prime factorizations of signal dimensions.
  vnl_fft_prime_factors<T> factors_[D];
};

#endif // vnl_fft_base_h_
