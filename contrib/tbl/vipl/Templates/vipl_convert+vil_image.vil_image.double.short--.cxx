#ifndef INSTANTIATE_TEMPLATES
#include <vipl/accessors/vipl_accessors_vil_image.txx>
#include <vipl/vipl_convert.txx>

template class vipl_convert<vil_image, vil_image, double, short, vipl_trivial_pixeliter>;
#endif
