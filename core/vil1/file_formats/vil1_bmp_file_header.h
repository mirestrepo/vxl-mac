#ifndef vil_bmp_file_header_h_
#define vil_bmp_file_header_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vil_bmp_file_header
// .INCLUDE vil/file_formats/vil_bmp_file_header.h
// .FILE file_formats/vil_bmp_file_header.cxx
// @author fsm@robots.ox.ac.uk

#include <vcl_iosfwd.h>
class vil_stream;

struct vil_bmp_file_header
{
  char     magic[2];      // 2
  unsigned file_size;     // 4
  short    reserved1;     // 2
  short    reserved2;     // 2
  unsigned bitmap_offset; // 4

  vil_bmp_file_header();
  void read(vil_stream *);
  void write(vil_stream *) const;
  void print(vcl_ostream &) const;
  bool signature_valid() const;
};

#endif // vil_bmp_file_header_h_
