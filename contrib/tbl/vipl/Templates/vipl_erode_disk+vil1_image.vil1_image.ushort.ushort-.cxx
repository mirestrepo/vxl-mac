#ifndef INSTANTIATE_TEMPLATES
#include <vipl/accessors/vipl_accessors_vil1_image.txx>
#include <vipl/vipl_erode_disk.txx>

template class vipl_erode_disk<vil1_image, vil1_image, unsigned short, unsigned short, vipl_trivial_pixeliter>;
#endif
