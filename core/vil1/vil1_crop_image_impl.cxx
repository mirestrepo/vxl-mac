#ifdef __GNUC__
#pragma implementation
#endif

// This is vxl/vil/vil_crop_image_impl.cxx

#include "vil_crop_image_impl.h"

vil_crop_image_impl::vil_crop_image_impl(vil_image const& gi, int x0, int y0, int w, int h):
  gi_(gi),
  x0_(x0),
  y0_(y0),
  width_(w),
  height_(h)
{
}

vil_crop_image_impl::~vil_crop_image_impl()
{
}

/* START_MANCHESTER_BINARY_IO_CODE */

  //: Return the name of the class;
const vcl_string& vil_crop_image_impl::is_a() const
{
  const static vcl_string class_name_="vil_crop_image_impl";
  return class_name_;
}

/* END_MANCHESTER_BINARY_IO_CODE */

