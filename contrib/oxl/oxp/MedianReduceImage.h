// This is oxl/oxp/MedianReduceImage.h
#ifndef MedianReduceImage_h_
#define MedianReduceImage_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//
// .NAME    MedianReduceImage - Oxford image processing
// .LIBRARY oxp
// .HEADER  Oxford Package
// .INCLUDE oxp/MedianReduceImage.h
// .FILE    MedianReduceImage.cxx
// .SECTION Description
//    POX is a collection of functions which are waiting for namespaces.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 13 Jul 98
//
//-----------------------------------------------------------------------------

#include <vil1/vil1_memory_image_of.h>

struct MedianReduceImage : public vil1_memory_image_of<unsigned char>
{
  MedianReduceImage(vil1_memory_image_of<unsigned char> const& in, int SCALE);
};

#endif // MedianReduceImage_h_
