// This is vxl/vnl/io/vnl_io_rational.txx
#ifndef vnl_io_rational_txx_
#define vnl_io_rational_txx_

#include <vsl/vsl_binary_io.h>
#include <vnl/io/vnl_io_rational.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_explicit_io.h>

//=================================================================================
//: Binary save self to stream.
void vsl_b_write(vsl_b_ostream & os, const vnl_rational & p)
{
  const short io_version_no = 1;
  vsl_b_write(os, io_version_no);
  vsl_b_write(os, p.numerator());
  vsl_b_write(os, p.denominator());
}

//=================================================================================
//: Binary load self from stream.
void vsl_b_read(vsl_b_istream &is, vnl_rational & p)
{
  if (!is) return;
  short ver;
  long n, d;
  vsl_b_read(is, ver);
  switch(ver)
  {
  case 1:
    vsl_b_read(is, n);
    vsl_b_read(is, d);
    p.set(n,d);
    break;

  default:
    vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&, vnl_rational&) \n";
    vcl_cerr << "           Unknown version number "<< ver << "\n";
    is.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    return;
  }
}

//====================================================================================
//: Output a human readable summary to the stream
void vsl_print_summary(vcl_ostream & os,const vnl_rational & p)
{
  os<<p;
}

#endif // vnl_io_rational_txx_
