#ifndef vnl_io_diag_matrix_h
#define vnl_io_diag_matrix_h
#ifdef __GNUC__
#pragma interface
#endif

//#include <vsl/vsl_binary_io.h>
#include <vnl/vnl_diag_matrix.h>

// This is vxl/vnl/io/vnl_io_diag_matrix.h

//:
// \file 
// \author dac
// \date 21-Mar-2001

//: Binary save vnl_real_polynomial to stream.
template <class T>
void vsl_b_write(vsl_b_ostream &os, const vnl_diag_matrix<T> & v);

//: Binary load vnl_real_polynomial from stream.
template <class T>
void vsl_b_read(vsl_b_istream &is, vnl_diag_matrix<T> & v);

//: Print human readable summary of object to a stream
template <class T>
void vsl_print_summary(vcl_ostream& os,const vnl_diag_matrix<T> & b);

#endif // vnl_io_diag_matrix_h



