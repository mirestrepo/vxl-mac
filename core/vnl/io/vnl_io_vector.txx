#ifndef vnl_io_vector_txx_
#define vnl_io_vector_txx_

// This is vxl/vnl/io/vnl_io_vector.txx

#include <vsl/vsl_binary_io.h>
#include <vnl/io/vnl_io_vector.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_explicit_io.h>

//=================================================================================
//: Binary save self to stream.
template<class T>
void vsl_b_write(vsl_b_ostream & os, const vnl_vector<T> & p)
{
  const short io_version_no = 1;
  vsl_b_write(os, io_version_no);
  vsl_b_write(os, p.size());
  vsl_b_write_block(os, p.begin(), p.size());

}

//=================================================================================
//: Binary load self from stream.
template<class T>
void vsl_b_read(vsl_b_istream &is, vnl_vector<T> & p)
{

  short ver;
  unsigned n;
  vsl_b_read(is, ver);
  switch(ver)
  {
  case 1:
    vsl_b_read(is, n);
    p.resize(n);
    vsl_b_read_block(is, p.begin(), n);
    break;

  default:
    vcl_cerr << "ERROR: vsl_b_read(s, vnl_vector&): Unknown version number "<< ver << vcl_endl;
    vcl_abort();
  }

}

//====================================================================================
//: Output a human readable summary to the stream
template<class T>
void vsl_print_summary(vcl_ostream & os,const vnl_vector<T> & p)
{
	os<<"Len: "<<p.size()<<" (";
    for ( int i =0; i < p.size() && i < 5; i++ )
		  os << p.operator()(i) <<" " ;
    if (p.size() > 5) os << " ...";
	os << ")" << vcl_endl;
}

#define VNL_IO_VECTOR_INSTANTIATE(T) \
template void vsl_print_summary(vcl_ostream &, const vnl_vector<T > &); \
template void vsl_b_read(vsl_b_istream &, vnl_vector<T > &); \
template void vsl_b_write(vsl_b_ostream &, const vnl_vector<T > &); \
;

#endif
