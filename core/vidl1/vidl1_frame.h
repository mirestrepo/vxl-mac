// This is core/vidl1/vidl1_frame.h
#ifndef vidl1_frame_h
#define vidl1_frame_h
//:
// \file
// \author Nicolas Dano, march 1999
//
// \verbatim
//  Modifications
//   Julien Esteve, May 2000 Ported from TargetJr
//   10/4/2001 Ian Scott (Manchester) Converted perceps header to doxygen
//   10/7/2003 Matt Leotta (Brown) Converted vil1 to vil
//   2004/09/10 Peter Vanroose - Added explicit copy constructor (ref_count !)
// \endverbatim

#include <vbl/vbl_ref_count.h>
#include <vil/vil_image_view_base.h>
#include <vidl1/vidl1_frame_sptr.h>
#include <vidl1/vidl1_codec.h>
#include <vil/vil_image_resource_sptr.h>

//: A single frame of a Video Sequence.
class vidl1_frame : public vbl_ref_count
{
  // PUBLIC INTERFACE
 public:

  // Constructors/Initializers/Destructors
  vidl1_frame(int position, vidl1_codec_sptr coder);
  vidl1_frame(vidl1_frame const& f)
    : vbl_ref_count(), position_(f.position_), coder_(f.coder_) {}
  ~vidl1_frame();

  //: Return an image resource
  vil_image_resource_sptr get_resource() const;
  vil_image_view_base_sptr get_view() const;
  virtual vil_image_view_base_sptr get_view(
                         int x0, int width, int y0, int height) const;

  vidl1_codec_sptr get_codec() const {return coder_;}

  inline char get_image_class()const { return coder_->get_image_class(); }
  inline char get_format() const     { return coder_->get_format(); }
  inline int  width() const          { return coder_->width(); }
  inline int  height() const         { return coder_->height(); }
  inline int  get_bits_pixel() const { return coder_->get_bits_pixel(); }
  inline int  get_bytes_pixel()const { return coder_->get_bytes_pixel(); }

  //: Return the index of this frame in the movie.
  int get_real_frame_index() const { return position_; }

 protected:

  // Data Members
  const int position_;
  vidl1_codec_sptr coder_;
};

#endif // vidl1_frame_h
