//-*- c++ -*-------------------------------------------------------------------
#ifndef vil_viff_h_
#define vil_viff_h_
#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \brief Loader for 1-banded Khoros 1.0 images
// \author    Peter.Vanroose@esat.kuleuven.ac.be
// \date 17 Feb 2000
//
//\verbatim
//  Modifications
//  3 October 2001 Peter Vanroose - Implemented get_property("top_row_first")
//\endverbatim

#include <vil/vil_file_format.h>
#include <vil/vil_image_impl.h>
#include "vil_viffheader.h"

//: Loader for VIFF files, i.e., 1-banded Khoros 1.0 images
// This supports char, short, int, float and double pixel cell types.
// No colour support right now.
// Adapted from the version by Reinhard Koch, Nov. 19, 1996.
class vil_viff_file_format : public vil_file_format {
public:
  virtual char const* tag() const;
  virtual vil_image_impl* make_input_image(vil_stream* vs);
  virtual vil_image_impl* make_output_image(vil_stream* vs, int planes,
                                            int width,
                                            int height,
                                            int components,
                                            int bits_per_component,
                                            vil_component_format format);
};

//: Generic image implementation for VIFF (Khoros) files
class vil_viff_generic_image : public vil_image_impl {
  vil_stream* is_;
  int width_;
  int height_;
  int maxval_;
  int planes_;

  int start_of_data_;
  int bits_per_component_;
  enum vil_component_format format_;

  bool endian_consistent_;
  vil_viff_xvimage header_;

  bool read_header();
  bool write_header();
  bool check_endian();

  friend class vil_viff_file_format;

public:
  vil_viff_generic_image(vil_stream* is);
  vil_viff_generic_image(vil_stream* is,
                         int planes,
                         int width,
                         int height,
                         int components,
                         int bits_per_component,
                         vil_component_format format);
  ~vil_viff_generic_image();

  //: Dimensions.  Planes x W x H x Components
  virtual int planes() const { return planes_; }
  virtual int width() const { return width_; }
  virtual int height() const { return height_; }
  virtual int components() const { return 1; }

  virtual int bits_per_component() const { return bits_per_component_; }
  virtual enum vil_component_format component_format() const { return format_; }

  //: Copy plane PLANE of this to BUF
  virtual bool get_section(void* buf, int x0, int y0, int width, int height) const;
  //: Write BUF into image plane PLANE of this
  virtual bool put_section(void const* buf, int x0, int y0, int width, int height);
  //: Emulate a single plane image
  virtual vil_image get_plane(int plane) const;

  //: Return the image interpreted as rgb bytes.
  virtual bool get_section_rgb_byte(void* buf, int x0, int y0, int width, int height) const;
  virtual bool get_section_float(void* buf, int x0, int y0, int width, int height) const;
  virtual bool get_section_byte(void* buf, int x0, int y0, int width, int height) const;

  char const* file_format() const;
  bool get_property(char const *tag, void *prop = 0) const;
};

#endif // vil_viff_h_
