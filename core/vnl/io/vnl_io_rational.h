// This is vxl/vnl/io/vnl_io_rational.h
#ifndef vnl_io_rational_h
#define vnl_io_rational_h
#ifdef __GNUC__
#pragma interface
#endif

#include <vnl/vnl_rational.h>

class vsl_b_ostream;
class vsl_b_istream;

//:
// \file 
// \author Peter Vanroose
// \date 10-Oct-2001

//: Binary save vnl_rational to stream.
void vsl_b_write(vsl_b_ostream & os, vnl_rational const& v);

//: Binary load vnl_rational from stream.
void vsl_b_read(vsl_b_istream & is, vnl_rational & v);

//: Print human readable summary of object to a stream
void vsl_print_summary(vcl_ostream & os, vnl_rational const& b);

#endif // vnl_io_rational_h
