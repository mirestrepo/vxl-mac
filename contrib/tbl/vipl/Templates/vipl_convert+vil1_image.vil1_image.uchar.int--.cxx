#ifndef INSTANTIATE_TEMPLATES
#include <vipl/accessors/vipl_accessors_vil1_image.txx>
#include <vipl/vipl_convert.txx>

template class vipl_convert<vil1_image, vil1_image, unsigned char, int, vipl_trivial_pixeliter>;
#endif
