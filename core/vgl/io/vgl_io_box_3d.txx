// This is vxl/vgl/io/vgl_io_box_3d.txx

#include <vgl/vgl_box_3d.h>
#include <vsl/vsl_binary_io.h>

//============================================================================
//: Binary save self to stream.
template<class T>
void vsl_b_write(vsl_b_ostream &os, const vgl_box_3d<T> & p)
{
  const short io_version_no = 1;
  vsl_b_write(os, io_version_no);
  vsl_b_write(os, p.get_min_x());
  vsl_b_write(os, p.get_min_y());
  vsl_b_write(os, p.get_min_z());
  vsl_b_write(os, p.get_max_x());
  vsl_b_write(os, p.get_max_y());
  vsl_b_write(os, p.get_max_z());
}

//============================================================================
//: Binary load self from stream.
template<class T>
void vsl_b_read(vsl_b_istream &is, vgl_box_3d<T> & p)
{
  short v;
  T min_pos[3];
  T max_pos[3];
  vsl_b_read(is, v);
  switch(v)
  {
  case 1:
    vsl_b_read(is, min_pos[0]);
    vsl_b_read(is, min_pos[1]);
    vsl_b_read(is, min_pos[2]);
    vsl_b_read(is, max_pos[0]);
    vsl_b_read(is, max_pos[1]);
    vsl_b_read(is, max_pos[2]);
    p.set_min_position(min_pos);
    p.set_max_position(max_pos);
    break;

  default:
    vcl_cerr << "vgl_box_3d<T>::vsl_b_read() ";
  vcl_cerr << "Unknown version number "<< v << vcl_endl;
    abort();
  }

}

//============================================================================
//: Output a human readable summary to the stream
template<class T>
void vsl_print_summary(vcl_ostream& os,const vgl_box_3d<T> & p)
{
  os<<"3d Box with opposite corners at (" <<p.get_min_x() << "," << 
    p.get_min_y() << "," << p.get_min_z() <<")" <<vcl_endl;
  os<<"and (" << p.get_max_x() << "," << p.get_max_y() << ","
    << p.get_max_z() << ")" <<vcl_endl;
}

#define VGL_IO_BOX_3D_INSTANTIATE(T) \
template void vsl_print_summary(vcl_ostream &, const vgl_box_3d<T> &); \
template void vsl_b_read(vsl_b_istream &, vgl_box_3d<T> &); \
template void vsl_b_write(vsl_b_ostream &, const vgl_box_3d<T> &); \
;
