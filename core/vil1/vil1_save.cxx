// This is ./vxl/vil/vil_save.cxx
#ifdef __GNUC__
#pragma implementation
#endif

//:
// \file

#include "vil_save.h"

#include <vcl_cstring.h>
#include <vcl_iostream.h>

#include <vil/vil_new.h>
#include <vil/vil_stream_fstream.h>
#include <vil/vil_image.h>
#include <vil/vil_copy.h>
#include <vil/vil_property.h> // for vil_property_top_row_first
#include <vil/vil_flipud.h>
#include <vil/vil_flip_components.h>

#include <vil/vil_rgb.h>
#include <vil/vil_memory_image_of.h>

//: Send vil_image to disk.
bool vil_save(vil_image i, char const* filename, char const* file_format)
{
  vil_stream_fstream* os = new vil_stream_fstream(filename, "w");
  vil_image out = vil_new(os, i.width(), i.height(), i, file_format);
  if (!out) {
    vcl_cerr << "vil_save: Cannot save to type [" << file_format << "]\n";
    return false;
  }
  bool top_first;
  if (out.get_property(vil_property_top_row_first, &top_first) && !top_first)
    i = vil_flipud(i);
  if (i.components() == 3 && out.get_property(vil_property_component_order_is_BGR))
    i = vil_flip_components(i);
  vil_copy(i, out);
  return true;
}

//: Send vil_image to disk; preserve byte order.
bool vil_save_raw(vil_image const& i, char const* filename, char const* file_format)
{
  vil_stream_fstream* os = new vil_stream_fstream(filename, "w");
  return vil_save_raw(i, os, file_format);
}

//: Send vil_image_impl to output stream.
// The possible file_formats are defined by the subclasses of vil_file_format
// in vil_file_format.cxx
bool vil_save_raw(vil_image const& i, vil_stream* os, char const* file_format)
{
  vil_image out = vil_new(os, i.width(), i.height(), i, file_format);

  if (!out) {
    vcl_cerr << "vil_save_raw: Cannot save to type [" << file_format << "]\n";
    return false;
  }

  vil_copy(i, out);

  return true;
}

//: save to file, deducing format from filename.
bool vil_save(vil_image const& i, char const* filename)
{
  char const *file_format = 0;

  // find last "."
  char const *dot = vcl_strrchr(filename, '.');
  if (!dot) {
    // filename doesn't end in ".anything"
    vcl_cerr << __FILE__ ": assuming pnm format for \'" << filename << "\'" << vcl_endl;
    file_format = "pnm";
  }
  else {
    // translate common extensions into known file formats.
    if (false) { }
#define macro(ext, fmt) else if (!vcl_strcmp(dot, "." #ext)) file_format = #fmt
    macro(bmp, bmp);
    macro(pbm, pnm);
    macro(pgm, pnm);
    macro(ppm, pnm);
    macro(pnm, pnm);
    macro(jpg, jpeg);
    macro(tif, tiff);
    macro(mit, mit);
    macro(gif, gif);
    macro(png, png);
#undef macro
    else {
      //file_format = dot+1; // hope it works.
      vcl_cerr << __FILE__ ": assuming pnm format for \'" << filename << "\'" << vcl_endl;
      file_format = "pnm";
    }
  }

  return vil_save(i, filename, file_format);
}

// What's the point of these *_template functions? Why not just put
// the function template declaration in the header file?

template<class T>
static inline
void vil_save_rgb_template(T const* p, int w, int h, vcl_string const& fn)
{
  vil_memory_image_of<vil_rgb<unsigned char> > out(w,h);
  unsigned char* o = (unsigned char*)out.get_buffer();
  T const* p_end = p + w*h*3;
  while (p != p_end)
    // possible loss of data! (e.g. clipping)
    // agap: but it's okay because the input values are in 0..255
    *o++ = (unsigned char)(*p++);
  vil_save(out, fn.c_str());
}

template<class T>
static inline
void vil_save_gray_template(T const* p, int w, int h, vcl_string const& fn)
{
  vil_memory_image_of<unsigned char> out(w,h);
  unsigned char* o = out.get_buffer();
  T const* p_end = p + w*h;
  while (p != p_end)
    // possible loss of data! (e.g. clipping)
    // agap: but it's okay because the input values are in 0..255
    *o++ = (unsigned char)(*p++);
  vil_save(out, fn.c_str());
}

//: Save raw unsigned chars, deducing format from filename
void vil_save_gray(unsigned char const* p, int w, int h, vcl_string const& fn)
{
  vil_save_gray_template(p, w, h, fn);
}

//: Save raw floats as gray.
// No scaling is performed, so values would be 0..255.
// File format is deduced from filename.
void vil_save_gray(float const* p, int w, int h, vcl_string const& fn)
{
  vil_save_gray_template(p, w, h, fn);
}

//: Save raw doubles as gray.
// No scaling is performed, so values would be 0..255.
// File format is deduced from filename.
void vil_save_gray(double const* p, int w, int h, vcl_string const& fn)
{
  vil_save_gray_template(p, w, h, fn);
}

//: Save raw RGB, deducing format from filename
void vil_save_rgb(unsigned char const* p, int w, int h, vcl_string const& fn)
{
  vil_save_rgb_template(p, w, h, fn);
}

//: Save raw floats as RGB.  See vil_save_gray.
void vil_save_rgb(float const* p, int w, int h, vcl_string const& fn)
{
  vil_save_rgb_template(p, w, h, fn);
}

//: Save raw doubles as RGB.  See vil_save_gray.
void vil_save_rgb(double const* p, int w, int h, vcl_string const& fn)
{
  vil_save_rgb_template(p, w, h, fn);
}
