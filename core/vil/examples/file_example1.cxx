//:
// \file
// \brief Example of creating and using an image data object
// \author Ian Scott

#include <vcl_iostream.h>
#include <testlib/testlib_root_dir.h>
#include <vil2/vil2_load.h>
#include <vil2/vil2_crop.h>
#include <vil2/vil2_image_view.h>
#include <vil2/vil2_image_view_functions.h>


int main(int argc, char** argv)
{

  vcl_string filename = testlib_root_dir() + "/vxl/vil/tests/file_read_data/ff_rgb8bit_raw.ppm";
  vcl_cout<<"Load " << filename << " into an image data object" << vcl_endl;


  // This is how we initialise an image data object.

  vil2_image_data_sptr data = vil2_load(filename.c_str());
  

  // This is how we get some image pixels from it.
  
  vil2_image_view_base * view = data->get_view(0,0,0,data->nx(), data->ny(), data->nplanes());

  vcl_cout << "Created a view of type " << view->is_a() << vcl_endl;

  vil2_image_view<unsigned char> uc_view= *view;

  vil2_print_all(vcl_cout,uc_view);

  // don't forget to delete view.
  delete view;
  
  return 0;
}
