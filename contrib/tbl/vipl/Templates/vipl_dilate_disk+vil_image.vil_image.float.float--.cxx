#ifndef INSTANTIATE_TEMPLATES
#include <vipl/accessors/vipl_accessors_vil_image.txx>
#include <vipl/vipl_dilate_disk.txx>

template class vipl_dilate_disk<vil1_image, vil1_image, float, float, vipl_trivial_pixeliter>;
#endif
