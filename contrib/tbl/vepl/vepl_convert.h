#ifndef vepl_convert_h_
#define vepl_convert_h_

//:
// \file
// \brief convert between arbitrary image types
//
//   This class actually copies one image to an other, pixel-wise.
//
// \author Peter Vanroose, K.U.Leuven (ESAT/PSI)
// \date   28 April 2001

#include <vil1/vil1_image.h>

//: convert between arbitrary image types
template <class D>
vil1_image vepl_convert(vil1_image const& , D dummy);

#ifdef INSTANTIATE_TEMPLATES
#include "vepl_convert.txx"
#endif

#endif // vepl_convert_h_
