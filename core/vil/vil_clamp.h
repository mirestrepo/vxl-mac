// This is core/vil2/vil2_clamp.h
#ifndef vil2_clamp_h_
#define vil2_clamp_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author Ian Scott.

#include <vil2/vil2_image_resource.h>
#include <vil2/vil2_image_view.h>
#include <vcl_cassert.h>


template <class T>
inline void vil2_clamp(vil2_image_view<T >&src, vil2_image_view<T >&dest, T lo, T hi)
{
  assert (hi >= lo);
  assert (src.nplanes() == dest.nplanes() &&
    src.nj() == dest.nj() &&
    src.ni() == dest.ni());
  for (unsigned p = 0; p < src.nplanes(); ++p)
    for (unsigned j = 0; j < src.nj(); ++j)
      for (unsigned i = 0; i < src.ni(); ++i)
      {
        const T v = dest(i,j,p);
        src(i,j,p) = v<lo?lo:(v>hi?hi:v);
      }
}


//: Clamp an image resource between two values.
// \relates vil2_image_resource
vil2_image_resource_sptr vil2_clamp(const vil2_image_resource_sptr &src, double low, double hi);


//: A generic_image adaptor that behaves like a clamped version of its input
// For implementation use only - use vil2_clamp() to create one.
class vil2_clamp_image_resource : public vil2_image_resource
{
  vil2_clamp_image_resource(vil2_image_resource_sptr const&, double low, double high);
  friend vil2_image_resource_sptr vil2_clamp(const vil2_image_resource_sptr &src, double low, double hi);
 public:

  virtual unsigned nplanes() const { return src_->nplanes(); }
  virtual unsigned ni() const { return src_->ni(); }
  virtual unsigned nj() const { return src_->nj(); }

  virtual enum vil2_pixel_format pixel_format() const { return src_->pixel_format(); }


  virtual vil2_image_view_base_sptr get_copy_view(unsigned i0, unsigned ni,
                                                  unsigned j0, unsigned nj) const;

  virtual vil2_image_view_base_sptr get_view(unsigned i0, unsigned ni,
                                             unsigned j0, unsigned nj) const {
    return get_copy_view(i0, ni, j0, nj); }


  //: Put the data in this view back into the image source.
  virtual bool put_view(const vil2_image_view_base& /*im*/, unsigned /*i0*/, unsigned /*j0*/) { return false; }

  //: Extra property information
  virtual bool get_property(char const* tag, void* property_value = 0) const;

 protected:
  //: Reference to underlying image source
  vil2_image_resource_sptr src_;
  //: Lower clamp value
  double lo_;
  //: Upper clamp value
  double hi_;
};

#endif // vil2_clamp_h_
