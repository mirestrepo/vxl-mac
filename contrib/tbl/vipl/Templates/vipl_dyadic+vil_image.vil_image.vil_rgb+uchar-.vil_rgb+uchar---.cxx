#ifndef INSTANTIATE_TEMPLATES
#include <vil/vil_rgb.h>
#include <vipl/accessors/vipl_accessors_vil_image.txx>
#include <vipl/vipl_dyadic.txx>

template class vipl_dyadic<vil_image, vil_image, vil_rgb<unsigned char>, vil_rgb<unsigned char>, vipl_trivial_pixeliter>;
#endif
