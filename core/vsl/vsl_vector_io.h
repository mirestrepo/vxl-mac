#ifndef vsl_vector_io_h_
#define vsl_vector_io_h_
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vsl/vsl_vector_io.h

//:
// \file 
// \brief binary IO functions for vcl_vector<T>
// \author Tim Cootes

#include <vcl_iosfwd.h>
#include <vcl_vector.h>

// Forward declaration
class vsl_b_ostream;
class vsl_b_istream;

//: Write vector to binary stream
template <class T>
void vsl_b_write(vsl_b_ostream& s, const vcl_vector<T>& v);

//: Read vector from binary stream
template <class T>
void vsl_b_read(vsl_b_istream& s, vcl_vector<T>& v);

//: Print human readable summary of object to a stream
template <class T>
void vsl_print_summary(vcl_ostream & os,const vcl_vector<T> &v);

#endif // vsl_vector_io_h_
