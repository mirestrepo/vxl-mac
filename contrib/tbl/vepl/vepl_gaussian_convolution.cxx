// This is tbl/vepl/vepl_gaussian_convolution.cxx
#include "vepl_gaussian_convolution.h"
#include <vcl_iostream.h>
#include <vipl/accessors/vipl_accessors_vil1_image.h>
#include <vipl/vipl_gaussian_convolution.h>
#include <vil1/vil1_memory_image_of.h>
#include <vil1/vil1_rgb.h>

vil1_image vepl_gaussian_convolution(vil1_image const& image, double sigma, double cutoff)
{
  // byte greyscale
  if (vil1_pixel_format(image) == VIL1_BYTE) {
    typedef unsigned char ubyte;
    vil1_memory_image_of<ubyte> mem(image); // load in memory to pass to filter
    vil1_memory_image_of<ubyte> out(image);
    vipl_gaussian_convolution<vil1_image,vil1_image,ubyte,ubyte> op(sigma, cutoff);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  // byte rgb: process colour bands independently as ubyte images
  else if (vil1_pixel_format(image) == VIL1_RGB_BYTE)
  {
    typedef unsigned char ubyte;
    typedef vil1_rgb<ubyte> r_g_b;
    vil1_memory_image_of<r_g_b> in(image); // load in memory to pass to filter
    vil1_memory_image_of<r_g_b> out(image);
    vil1_memory_image_of<ubyte> mem((ubyte*)(in.get_buffer()),3*in.width(),in.height()); // reinterpret as ubyte
    vil1_memory_image_of<ubyte> mout((ubyte*)(out.get_buffer()),3*in.width(),in.height());
    vipl_gaussian_convolution<vil1_image,vil1_image,ubyte,ubyte> op(sigma, cutoff);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&mout);
    op.filter();
    return out;
  }


  // 16-bit greyscale
  else if (vil1_pixel_format(image) == VIL1_UINT16) {
    typedef unsigned short ushort;
    vil1_memory_image_of<ushort> mem(image); // load in memory to pass to filter
    vil1_memory_image_of<ushort> out(image);
    vipl_gaussian_convolution<vil1_image,vil1_image,ushort,ushort> op(sigma, cutoff);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  // float
  else if (vil1_pixel_format(image) == VIL1_FLOAT) {
    vil1_memory_image_of<float> mem(image); // load in memory to pass to filter
    vil1_memory_image_of<float> out(image);
    vipl_gaussian_convolution<vil1_image,vil1_image,float,float> op(sigma, cutoff);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  // double
  else if (vil1_pixel_format(image) == VIL1_DOUBLE) {
    vil1_memory_image_of<double> mem(image); // load in memory to pass to filter
    vil1_memory_image_of<double> out(image);
    vipl_gaussian_convolution<vil1_image,vil1_image,double,double> op(sigma, cutoff);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  //
  else {
    vcl_cerr << __FILE__ ": vepl_gaussian_convolution() not implemented for " << image << vcl_endl;
    return 0;
  }
}

