#ifndef vbl_io_bounding_box_h
#define vbl_io_bounding_box_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vbl/vbl_io_bounding_box_3d.h

//:
// \file
// \author P D Allen
// \date 21-Mar-2001

#include <vsl/vsl_binary_io.h>
#include <vbl/vbl_bounding_box.h>

//: Binary save vbl__point_3d to stream.
template <class T, class DIM_>
void vsl_b_write(vsl_b_ostream &os, const vbl_bounding_box_base<T, DIM_> & p);

//: Binary load vbl_bounding_box from stream.
template <class T, class DIM_>
void vsl_b_read(vsl_b_istream &is, vbl_bounding_box_base<T, DIM_> & p);

//: Print human readable summary of object to a stream
template <class T, class DIM_>
void vsl_print_summary(vcl_ostream& os,const vbl_bounding_box_base<T, DIM_> & p);


#endif // vbl_io_bounding_box_h
