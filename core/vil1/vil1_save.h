#ifndef vil_save_h_
#define vil_save_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vil_save
// .INCLUDE vil/vil_save.h
// .FILE vil_save.cxx
// .SECTION Author
//    awf@robots.ox.ac.uk
// Created: 16 Feb 00
//
// .SECTION Modifications
//     000216 AWF Initial version.

#include <vcl_string.h>
class vil_stream;
#include <vil/vil_image.h>

//: Send vil_image to disk, deducing format from filename
bool vil_save(vil_image const&, char const* filename);

//: Send vil_image to disk, given filename
bool vil_save(vil_image const&, char const* filename, char const* file_format);

//: Send vil_image to outstream
bool vil_save(vil_image const &, vil_stream* outstream, char const* file_format = "pnm");

//: Save raw unsigned chars, deducing format from filename
void vil_save_gray(unsigned char const* p, int w, int h, vcl_string const& fn);

//: Save raw floats as gray.
// No scaling is performed, so values whould be 0..255.
// File format is deduced from filename.
void vil_save_gray(float const* p, int w, int h, vcl_string const& fn);

//: Save raw doubles as gray.
// No scaling is performed, so values whould be 0..255.
// File format is deduced from filename.
void vil_save_gray(double const* p, int w, int h, vcl_string const& fn);


//: Save raw RGB, deducing format from filename
void vil_save_rgb(unsigned char const* p, int w, int h, vcl_string const& fn);

//: Save raw floats as RGB.  See vil_save_gray.
void vil_save_rgb(float const* p, int w, int h, vcl_string const& fn);

//: Save raw doubles as RGB.  See vil_save_gray.
void vil_save_rgb(double const* p, int w, int h, vcl_string const& fn);

#endif // vil_save_h_
