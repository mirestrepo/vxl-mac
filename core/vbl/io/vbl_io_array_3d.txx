// This is vxl/vbl/io/vbl_io_array_3d.txx
#ifndef vbl_io_array_3d_txx_
#define vbl_io_array_3d_txx_

//:
// \file
// \brief  binary IO functions for vbl_array_3d<T>
// \author K.Y.McGaul

#include <vsl/vsl_binary_io.h>
#include <vbl/vbl_array_3d.h>


//====================================================================
//: Binary save self to stream.
template<class T>
void vsl_b_write(vsl_b_ostream & os, const vbl_array_3d<T> &p)
{
  const short io_version_no = 1;
  vsl_b_write(os, io_version_no);

  int row1_count = p.get_row1_count();
  int row2_count = p.get_row2_count();
  int row3_count = p.get_row3_count();
  vsl_b_write(os, row1_count);
  vsl_b_write(os, row2_count);
  vsl_b_write(os, row3_count);
  for (unsigned i=0; i<row1_count; i++)
    for (unsigned j=0; j<row2_count; j++)
      for (unsigned k=0; k<row3_count; k++)
        vsl_b_write(os, p(i,j,k));
}

//=====================================================================
//: Binary load self from stream.
template<class T>
void vsl_b_read(vsl_b_istream &is, vbl_array_3d<T> &p)
{
  short ver;
  int row1_count, row2_count, row3_count;
  vsl_b_read(is, ver);
  switch(ver)
  {
  case 1:
    vsl_b_read(is, row1_count);
    vsl_b_read(is, row2_count);
    vsl_b_read(is, row3_count);
    p.resize(row1_count, row2_count, row3_count);
    for (unsigned i=0; i<row1_count; i++)
      for (unsigned j=0; j<row2_count; j++)
        for (unsigned k=0; k<row3_count; k++)
          vsl_b_read(is, p(i,j,k));
    break;

  default:

    vcl_cerr << "ERROR: vsl_b_read(s, vbl_array_3d&): Unknown version number "<<
                                                   ver << vcl_endl;
    vcl_abort();
  }
}

//====================================================================
//: Output a human readable summary to the stream
template<class T>
void vsl_print_summary(vcl_ostream & os,const vbl_array_3d<T> & p)
{
  os << "Row1 count: " << p.get_row1_count() << vcl_endl;
  os << "Row2 count: " << p.get_row2_count() << vcl_endl;
  os << "Row3 count: " << p.get_row3_count() << vcl_endl;
  for (unsigned i =0; i<p.get_row1_count() && i<5; i++)
  {
    for (unsigned j=0; j<p.get_row2_count() && j<5; j++)
    {
      for (unsigned k=0; k<p.get_row3_count() && k<5; k++)
      {
        os << " ";
        vsl_print_summary(os, p(i,j,k));
      }
      if (p.get_row3_count() > 5)
        os << " ..." << vcl_endl;
    }
    if (p.get_row2_count() > 5)
      os << " ...";
    os << vcl_endl;
    os << vcl_endl;
  }
  if (p.get_row1_count() > 5)
    os << " ..." << vcl_endl;
}

#define VBL_IO_ARRAY_3D_INSTANTIATE(T) \
template void vsl_print_summary(vcl_ostream &, const vbl_array_3d<T > &); \
template void vsl_b_read(vsl_b_istream &, vbl_array_3d<T > &); \
template void vsl_b_write(vsl_b_ostream &, const vbl_array_3d<T > &)

#endif // vbl_io_array_3d_txx_
