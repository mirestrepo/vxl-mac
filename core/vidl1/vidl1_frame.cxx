// This is core/vidl1/vidl1_frame.cxx
#include "vidl1_frame.h"
//:
// \file

#include <vidl1/vidl1_codec_sptr.h>
#include <vil/vil_image_view_base.h>

//=========================================================================
//  Methods for vidl1_frame.
//_________________________________________________________________________

//------------------------------------------------------------------------
// CONSTRUCTOR(S) AND DESTRUCTOR

//: Constructor, needs the frame position in the clip and the coder used.
vidl1_frame::vidl1_frame(int position, vidl1_codec_sptr coder) :
position_(position), coder_(coder)
{
}

//: Destructor
vidl1_frame::~vidl1_frame()
{
}

//: Return the image.
vil_image_view_base_sptr vidl1_frame::get_view() const
{
  return coder_->get_view(position_);
}


//: Return an image resource
vil_image_resource_sptr vidl1_frame::get_resource() const
{
  return coder_->get_resource(position_);
}


//: Get the pixels for the rectangular window starting at x0, y0 and width and height wide.
vil_image_view_base_sptr vidl1_frame::get_view(int x0, int width, int y0, int height) const
{
  return coder_->get_view(position_, x0, width, y0, height);
}

