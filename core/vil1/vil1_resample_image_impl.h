#ifndef vil_resample_image_impl_h_
#define vil_resample_image_impl_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vil_resample_image_impl
// .INCLUDE vil/vil_resample_image_impl.h
// .FILE vil_resample_image_impl.cxx
// \author fsm@robots.ox.ac.uk

#include <vil/vil_image_impl.h>
#include <vil/vil_image.h>

//: Adaptor which produces a image by resampling.
class vil_resample_image_impl : public vil_image_impl {
public:
  vil_resample_image_impl(vil_image const &underlying, unsigned nw, unsigned nh);
  ~vil_resample_image_impl();
  
  //: these inlines partly document the semantics of vil_resample_image.
  int planes() const { return base.planes(); }
  int width() const { return new_width; }
  int height() const { return new_height; }
  int components() const { return base.components(); }
  int bits_per_component() const { return base.bits_per_component(); }
  vil_component_format component_format() const { return base.component_format(); }
  
  vil_image get_plane(int ) const;
  
  bool get_section(void *buf, int x0, int y0, int w, int h) const;
  bool put_section(void const *buf, int x0, int y0, int w, int h); // <- will fail
  
  bool get_property(char const *tag, void *property_value_out = 0) const;
  
private:
  vil_image base;
  unsigned new_width, new_height;
};

#endif // vil_resample_image_impl_h_
