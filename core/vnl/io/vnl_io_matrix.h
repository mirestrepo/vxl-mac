#ifndef vnl_io_matrix_h
#define vnl_io_matrix_h
#ifdef __GNUC__
#pragma interface
#endif



// This is vxl/vnl/io/vnl_io_matrix.h

//:
// \file 
// \author Louise Butcher
// \date 20-Mar-2001


#include <vsl/vsl_binary_io.h>
#include <vnl/vnl_matrix.h>

//: Binary save vnl_matrix to stream.
template <class T>
void vsl_b_write(vsl_b_ostream & os, const vnl_matrix<T> & v);

//: Binary load vnl_matrix from stream.
template <class T>
void vsl_b_read(vsl_b_istream & is, vnl_matrix<T> & v);

//: Print human readable summary of object to a stream
template <class T>
void vsl_print_summary(vcl_ostream & os,const vnl_matrix<T> & b);


#endif // vnl_io_matrix_h
