// This is core/vil/vil_open.h
#ifndef vil_open_h_
#define vil_open_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief make a vil_stream from a filename, an URL, etc.
//
// \author fsm

#include <vil/vil_stream.h>

//: make a vil_stream from a filename, an URL, etc.
// \relates vil_stream
vil_stream *vil_open(char const* what, char const* how = "r");

#endif // vil_open_h_
