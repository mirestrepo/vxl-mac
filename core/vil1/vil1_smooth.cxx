// This is vxl/vil/vil_smooth.cxx

/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vil_smooth.h"

#include <vcl_iostream.h>
#include <vcl_cmath.h>
#include <vcl_vector.h>

#include <vil/vil_memory_image_of.h>
#include <vil/vil_image_as.h>
#include <vil/vil_convolve.h>
#include <vil/vil_rgb.h>
#include <vil/vil_new.h>

#include <vil/vil_convolve.txx>

vil_image vil_smooth_gaussian(vil_image const & in, double sigma)
{
  // Create 1-D mask:
  double cutoff = 0.01;
  double lc = -2 * vcl_log(cutoff); // cutoff guaranteed > 0
  int radius = (lc<=0) ? 0 : 1 + int(vcl_sqrt(lc)*sigma); // sigma guaranteed >= 0
  int size = 2*radius + 1;
  vcl_vector<float> mask(size);
  double halfnorm = 0.5;
  mask[radius] = 1.0;
  for (int x=1; x<=radius; ++x) {
    double v = vcl_exp(-0.5*x*x/(sigma*sigma));
    mask[radius - x] = mask[radius + x] = v;
    halfnorm += 2*v;
  }

  // normalise mask
  double mass_scale = 1.0/(1 + 2*halfnorm);
  for (int x=0; x<=2*size+1; ++x)
    mask[x] *= mass_scale;

  // Call convolver
  if (vil_pixel_format(in) == VIL_BYTE)
    return vil_convolve_separable(in, /* xxx */&mask[0], size-1, (vil_byte*)0, (float*)0);

// if (vil_pixel_type(in) == VIL_RGB_BYTE)
//    return vil_convolve_separable(in, mask.begin(), size, (vil_rgb_byte*)0, (vil_rgb<float>*)0);

  return 0;
}

#define inst(pixel_type, float_type) \
template \
void vil_convolve_separable(float const kernel[], unsigned N, \
                                 vil_memory_image_of<pixel_type> & buf, \
                                 vil_memory_image_of<float_type>& tmp, \
                                 vil_memory_image_of<float_type>& out); \
template \
vil_image vil_convolve_separable(vil_image const &, float const*, int, pixel_type*, float_type* )

inst(unsigned char, float);
inst(int, float);
