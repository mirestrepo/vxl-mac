// This is core/vil/vil_image_view_base.h
#ifndef vil_image_view_base_h_
#define vil_image_view_base_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief A base class reference-counting view of some image data.
// \author Ian Scott - Manchester

#include <vcl_iosfwd.h>
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vil/vil_pixel_format.h>
#include <vil/vil_smart_ptr.h>

//: An abstract base class of smart pointers to actual image data in memory.
// If you want an actual image, try instantiating vil_image_view<T>.

class vil_image_view_base
{
 protected:
  //: Number of columns.
  unsigned ni_;
  //: Number of rasters.
  unsigned nj_;
  //: Number of planes.
  unsigned nplanes_;

  vil_image_view_base(unsigned ni, unsigned nj, unsigned nplanes):
  ni_(ni), nj_(nj), nplanes_(nplanes), reference_count_(0) {}

  //: Default is an empty one-plane image
  //  Don't set nplanes_ to zero as it confuses set_size(nx,ny) later
  vil_image_view_base(): ni_(0), nj_(0), nplanes_(1), reference_count_(0) {}

 public:
  // The destructor must be virtual so that the memory chunk is destroyed.
  virtual ~vil_image_view_base() {};

  //: Width
  unsigned ni()  const {return ni_;}
  //: Height
  unsigned nj()  const {return nj_;}
  //: Number of planes
  unsigned nplanes() const {return nplanes_;}

  //: The number of pixels.
  unsigned long size() const { return ni_ * nj_ * nplanes_; }

  //: set_size current planes to width x height.
  // If already correct size, this function returns quickly
  virtual void set_size(unsigned width, unsigned height) =0;

  //: resize to width x height x nplanes.
  // If already correct size, this function returns quickly
  virtual void set_size(unsigned width, unsigned height, unsigned nplanes) =0;

  //: Print a 1-line summary of contents
  virtual void print(vcl_ostream&) const =0;

  //: Return class name
  virtual vcl_string is_a() const =0;

  //: Return a description of the concrete data pixel type.
  // For example if the value is VIL_PIXEL_FORMAT_BYTE,
  // you can safely cast, or assign the base class reference to
  // a vil_image_view<vxl_byte>.
  virtual enum vil_pixel_format pixel_format() const=0;

  //: True if this is (or is derived from) class s
  virtual bool is_class(vcl_string const& s) const;

 private:
  // You probably should not use a vil_image_view in a vbl_smart_ptr, so the
  // ref functions are private
  friend class vil_smart_ptr<vil_image_view_base>;
  void ref() { ++reference_count_; }
  void unref() {
    assert(reference_count_>0);
    if (--reference_count_<=0) delete this;}
  int reference_count_;
};

typedef vil_smart_ptr<vil_image_view_base> vil_image_view_base_sptr;

//: Print a 1-line summary of contents
inline
vcl_ostream& operator<<(vcl_ostream& s, vil_image_view_base const& im) {
  im.print(s); return s;
}

#endif // vil_image_view_base_h_
