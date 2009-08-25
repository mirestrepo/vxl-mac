#include "bvpl_kernel.h"
//:
// \file

#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vcl_fstream.h>
#include <vcl_string.h>

unsigned bvpl_kernel::id_cnt=0;

//: Saves the kernel to ascii file
void bvpl_kernel::print_to_file(vcl_string filename)
{
  vcl_fstream ofs(filename.c_str(), vcl_ios::out);
  if (!ofs.is_open()) {
    vcl_cerr << "error opening file " << filename << " for write!\n";
    return;
  }
  
  kernel_.begin();
  while (!kernel_.isDone()){
    vgl_point_3d<int> coord = kernel_.index();
    ofs.precision(2);
    ofs << coord.x() << ' ' << coord.y() << ' ' << coord.z() << ' ' << (*kernel_).c_ << "\n" ; 
    ++kernel_;
  }
  
  ofs.close();
}

//: Saves the kernel to Dristhi .raw data format.
// The kernel does not occupy the entire volume, so the empty voxels are set to 0.
// The size of the box is max(x,y,z) * max(x,y,z) * max(x,y,z)
bool bvpl_kernel::save_raw(vcl_string filename)
{
  vcl_fstream ofs(filename.c_str(), vcl_ios::binary | vcl_ios::out);
  if (!ofs.is_open()) {
    vcl_cerr << "error opening file " << filename << " for write!\n";
    return false;
  }
  
  // write header
  //unsigned char data_type = 1; // 1 means signed byte
  unsigned char data_type = 8; // 8 means float
  
  vxl_uint_32 nx = (max_.x() - min_.x()) + 1;
  vxl_uint_32 ny = (max_.y() - min_.y()) + 1;
  vxl_uint_32 nz = (max_.z() - min_.z()) + 1;
  
  ofs.write(reinterpret_cast<char*>(&data_type),sizeof(data_type));
  ofs.write(reinterpret_cast<char*>(&nx),sizeof(nx));
  ofs.write(reinterpret_cast<char*>(&ny),sizeof(ny));
  ofs.write(reinterpret_cast<char*>(&nz),sizeof(nz));
  
  // write data
  // iterate through slabs and fill in memory array
  unsigned size = nx*ny*nz;
  float *data_array = new float[size];
  
  kernel_.begin();
  if (!kernel_.isDone())
  {
    float max = (*kernel_).c_,
    min = (*kernel_).c_;
    ++kernel_;
    
    while (!kernel_.isDone()) {
      if ((*kernel_).c_> max)
        max = (*kernel_).c_;
      if ((*kernel_).c_< min)
        min = (*kernel_).c_;
      ++kernel_;
    }
    vcl_cout << "max: " << max <<vcl_endl
    << "min: " << min <<vcl_endl;
  }
  
  //Since our kernel does not occupy the entire space we need to initialize our data
  for (unsigned i = 0; i < size; i++)
    data_array[i] = 0;
  
  kernel_.begin();
  while (!kernel_.isDone()){
    vgl_point_3d<int> coord = kernel_.index();
    int index = (coord.x()-min_.x())*ny*nz + (coord.y()-min_.y())*nz + (coord.z() - min_.z());
    data_array[index] = (float)((*kernel_).c_);
    ++kernel_;
  }
  vcl_cout << vcl_endl;
  
  ofs.write(reinterpret_cast<char*>(data_array),sizeof(float)*nx*ny*nz);
  ofs.close();
  
  delete[] data_array;
  return true;
}
