#ifndef vgl_io_point_2d_h
#define vgl_io_point_2d_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vgl/io/vgl_io_point_2d.h

#include <vgl/vgl_point_2d.h>
#include <vsl/vsl_binary_io.h>

//:
// \file
// \author Tim Cootes
// \date 19-Mar-2001

//: Binary save vgl_point_2d to stream.
template <class T>
void vsl_b_write(vsl_b_ostream &os, const vgl_point_2d<T> & v);

//: Binary load vgl_point_2d from stream.
template <class T>
void vsl_b_read(vsl_b_istream &is, vgl_point_2d<T> & v);

//: Print human readable summary of object to a stream
template <class T>
void vsl_print_summary(vcl_ostream& os,const vgl_point_2d<T> & b);


#endif // vgl_io_point_2d_h
