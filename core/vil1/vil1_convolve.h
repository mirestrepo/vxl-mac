#ifndef vil_convolve_h_
#define vil_convolve_h_
// This is vxl/vil/vil_convolve.h

//:
// \file
// \author fsm@robots.ox.ac.uk

#include <vcl_compiler.h>

//: Available options for boundary behavior
// When convolving a finite signal the boundaries may be
// treated in various ways which can often be expressed in terms
// of ways to extend the signal outside its original range.
enum vil_convolve_boundary_option {
  // Do not to extend the signal, but pad with zeros.
  //     |                               |
  // K                       ----*-------
  // in   ... ---------------------------
  // out  ... --------------------0000000
  vil_convolve_no_extend,

  // Zero-extend the input signal beyond the boundary.
  //     |                               |
  // K                              ----*--------
  // in   ... ---------------------------000000000000...
  // out  ... ---------------------------
  vil_convolve_zero_extend,

  // Extend the signal to be constant beyond the boundary
  //     |                               |
  // K                              ----*--------
  // in   ... --------------------------aaaaaaaaaaaaa...
  // out  ... ---------------------------
  vil_convolve_constant_extend,

  // Extend the signal periodically beyond the boundary.
  //     |                               |
  // K                              ----*--------
  // in   abc...-------------------------abc...------..
  // out  ... ---------------------------
  vil_convolve_periodic_extend,

  // Extend the signal by reflection about the boundary.
  //     |                               |
  // K                               ----*--------
  // in   ... -------------------...edcbabcde...
  // out  ... ---------------------------
  vil_convolve_reflect_extend,

  // This one is slightly different. The input signal is not
  // extended in any way, but the kernel is trimmed to allow
  // convolution to proceed up to the boundary and reweighted
  // to keep the total area the same.
  // *** may not work with kernels which take negative values.
  vil_convolve_trim
};

//: Parameters for convolution
// These structs exist purely to group the parameters to the
// convolution routines. It is not intended that they be
// expressive or even useful in any other context.
//
// Usually, begin <= origin < end. Expect assertion failures
// if that is not the case.
template <class T>
struct vil_convolve_signal_1d {
  T *array;
  int begin;
  int origin;
  int end;
  vil_convolve_signal_1d(T *a, int b, int o, int e)
    : array(a), begin(b), origin(o), end(e) { }
};

//: Parameters for convolution
template <class T>
struct vil_convolve_signal_2d {
  T * const *array;
  int beginx, originx, endx;
  int beginy, originy, endy;
  vil_convolve_signal_2d(T * const *a,
                         int bx, int ox, int ex,
                         int by, int oy, int ey)
    : array(a)
    , beginx(bx), originx(ox), endx(ex)
    , beginy(by), originy(oy), endy(ey)
    { }
};

// Note. The convolution operation is defined by
//    (f*g)(x) = \int f(x-y) g(y) dy,
// i.e. one operand is reflected before the integration is performed.
// If you don't want this to happen, the behaviour you want is not
// called "convolution". So don't break the convolution routines in
// that particular way.

//: Convolution in x-direction : out(x, y) = \sum_i kernel[i]*in(x-i, y)
template <class I1, class I2, class AC, class O>
void vil_convolve_1d_x(vil_convolve_signal_1d<I1 const> const &kernel,
                       vil_convolve_signal_2d<I2 const> const &input,
                       AC * /*accumulator type*/,
                       vil_convolve_signal_2d<O> const &output,
                       vil_convolve_boundary_option b,
                       vil_convolve_boundary_option e);

//: Convolution in y-direction : out(x, y) = \sum_j kernel[j]*in(x, y-j)
template <class I1, class I2, class AC, class O>
void vil_convolve_1d_y(vil_convolve_signal_1d<I1 const> const &kernel,
                       vil_convolve_signal_2d<I2 const> const &input,
                       AC * /*accumulator type*/,
                       vil_convolve_signal_2d<O> const &output,
                       vil_convolve_boundary_option b,
                       vil_convolve_boundary_option e);

// *** the following are not yet implemented.

//: Convolution in x-direction, using a symmetric kernel.
template <class I1, class I2, class AC, class O>
void vil_convolve_1d_x(I1 const *half_kernel, unsigned kernel_size,
                       vil_convolve_signal_2d<I2 const> const &input,
                       AC * /*accumulator type*/,
                       vil_convolve_signal_2d<O> const &output,
                       vil_convolve_boundary_option b,
                       vil_convolve_boundary_option e);

//: Convolution in y-direction, using a symmetric kernel.
template <class I1, class I2, class AC, class O>
void vil_convolve_1d_y(I1 const *half_kernel, unsigned kernel_size,
                       vil_convolve_signal_2d<I2 const> const &input,
                       AC * /*accumulator type*/,
                       vil_convolve_signal_2d<O> const &output,
                       vil_convolve_boundary_option b,
                       vil_convolve_boundary_option e);

#endif // vil_convolve_h_
