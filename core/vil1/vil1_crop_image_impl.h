//-*- c++ -*-------------------------------------------------------------------
#ifndef vil_crop_image_h_
#define vil_crop_image_h_
#ifdef __GNUC__
#pragma interface
#endif
// Author: awf@robots.ox.ac.uk
// Created: 16 Feb 00

#include <vil/vil_image_impl.h>
#include <vil/vil_image.h>
//#include <vcl/vcl_vector.h>

//: A generic_image adaptor that behaves like a cropped version of its input
class vil_crop_image_impl : public vil_image_impl {
public:
  vil_crop_image_impl(vil_image const&, int x0, int y0, int w, int h);
  ~vil_crop_image_impl();

  int planes() const { return gi_.planes(); }
  int width() const { return width_; }
  int height() const { return height_; }
  int components() const { return gi_.components(); }

  int bits_per_component() const { return gi_.bits_per_component(); }
  enum vil_component_format component_format() const { return gi_.component_format(); }
  
  bool get_section(void* buf, int x0, int y0, int width, int height) const {
    return gi_.get_section(buf, x0 + x0_, y0 + y0_, width, height);
  }
  bool put_section(void const* buf, int x0, int y0, int width, int height) {
    return gi_.put_section(buf, x0 + x0_, y0 + y0_, width, height);
  }
  //  vil_image get_plane(int ) const;

protected:
  vil_image gi_;
  int x0_;
  int y0_;
  int width_;
  int height_;
  //  vcl_vector<vil_crop_image_impl*> plane_crops_;
};

#endif   // DO NOT ADD CODE AFTER THIS LINE! END OF DEFINITION FOR CLASS vil_crop_image_impl.
