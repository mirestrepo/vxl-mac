#ifndef vcl_sstream_h_
#define vcl_sstream_h_
/*
  fsm@robots.ox.ac.uk
*/

// this is to get the vcl_ios_* macros.
#include "vcl_iostream.h"

#if (defined(VCL_GCC) && !defined(GNU_LIBSTDCXX_V3)) || defined(VCL_SGI_CC_720)
# include <strstream.h>
# include <vcl_string.h>

# undef  vcl_stringstream
# define vcl_stringstream vcl_stringstream
struct vcl_stringstream : strstream
{
  vcl_stringstream(int /*which*/ = vcl_ios_in | vcl_ios_out) { }
  //FIXME vcl_stringstream(vcl_string const &s, int /*which*/ = vcl_ios_in | vcl_ios_out) { }
  
  // [27.7.4]
  vcl_string str() { return strstream::str(); }
  //FIXME void str(vcl_string const &s);
};

# undef  vcl_istringstream
# define vcl_istringstream vcl_istringstream
struct vcl_istringstream : istrstream
{
  vcl_istringstream(vcl_string const &s) : istrstream(s.c_str()) { }
  
  // [27.7.2.2]
  vcl_string str() { return istrstream::rdbuf()->str(); }
  //FIXME void str(vcl_string const &s);
};

# undef  vcl_ostringstream
# define vcl_ostringstream vcl_ostringstream
struct vcl_ostringstream : ostrstream
{
  // [27.7.3.2]
  vcl_string str() { return ostrstream::str(); }
  //FIXME void str(vcl_string const &s);
};

#else
# include "iso/vcl_sstream.h"
#endif

#endif // vcl_sstream_h_
