#ifndef vul_io_user_info_h
#define vul_io_user_info_h
#ifdef __GNUC__
#pragma interface
#endif

#include <vsl/vsl_binary_io.h>
#include <vul/vul_user_info.h>

// This is vxl/vul/io/vul_io_user_info.h

//:
// \file
// \brief binary io for vul_user_info.
// \author Christine Beeston
// \date 22-Mar-2001

//: Binary save vul_user_info to stream.
void vsl_b_write(vsl_b_ostream &os, const vul_user_info & v);

//: Binary load vul_user_info from stream.
void vsl_b_read(vsl_b_istream &is, vul_user_info & v);

//: Print human readable summary of object to a stream
void vsl_print_summary(vcl_ostream& os,const vul_user_info & v);


#endif // vul_io_user_info_h
