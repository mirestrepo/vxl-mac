#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vil_scale_intensities
// Author: David Capel, Oxford RRG
// Created: 26 Jul 2000
//
//-----------------------------------------------------------------------------

#include "vil_scale_intensities.h"
#include <vil/vil_scale_intensities_image_impl.h>

vil_image vil_scale_intensities(vil_image src, double scale, double shift) {
  return new vil_scale_intensities_image_impl(src, scale, shift);
}
