#ifndef vil_crop_h_
#define vil_crop_h_
#ifdef __GNUC__
#pragma interface
#endif
// This is vxl/vil/vil_crop.h

//:
// \file
// \author awf@robots.ox.ac.uk
// \date 16 Feb 00

#include <vil/vil_fwd.h>

//: Crop to a region of SRC.
vil_image vil_crop(vil_image SRC, int x0, int y0, int w, int h);

#endif // vil_crop_h_
