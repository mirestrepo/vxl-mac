#ifndef vil_stream_section_h_
#define vil_stream_section_h_
#ifdef __GNUC__
#pragma interface
#endif
// This is vxl/vil/vil_stream_section.h

//:
// \file
// \brief make a section of a vil_stream behave like a vil_stream
// \author  fsm@robots.ox.ac.uk

#include <vil/vil_stream.h>

//: make a section of a vil_stream behave like a vil_stream.
//
// It is possible to have multiple vil_stream_sections using the same
// underlying stream simultaneously. This is accomplished by keeping
// a note of the current position and seeking a lot.
//
// Note however that this is *not* threadsafe.
struct vil_stream_section : vil_stream
{
  // skip to position 'begin' in underlying stream and translate seeks,
  // reads and writes relative to that position into seeks, reads and
  // writes in the underlying stream.
  vil_stream_section(vil_stream *underlying, int begin);

  // as above, but will not allow seeks, reads or writes past 'end'.
  vil_stream_section(vil_stream *underlying, int begin, int end);

  bool ok();
  int write(void const* buf, int n);
  int read(void* buf, int n);
  int  tell();
  void seek(int position);

protected:
  ~vil_stream_section();

private:
  vil_stream *underlying_;
  int begin_;
  int end_;
  int current_;
};

#endif // vil_stream_section_h_
