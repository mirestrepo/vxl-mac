#include <vcl_iostream.h>
#include <vipl/accessors/vipl_accessors_vil_image.h>
#include <vepl/vepl_threshold.h>
#include <vipl/vipl_threshold.h>
#include <vil/vil_memory_image_of.h>

vil_image vepl_threshold(vil_image const& image, double threshold, double below, double above)
{
  // byte greyscale
  if (vil_pixel_format(image) == VIL_BYTE) {
    typedef unsigned char ubyte;
    vil_memory_image_of<ubyte> mem(image); // load in memory to pass to filter
    vil_memory_image_of<ubyte> out(image);
    vipl_threshold<vil_image,vil_image,ubyte,ubyte,vipl_trivial_pixeliter> op((ubyte)threshold, (ubyte)below, (ubyte)above);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  // byte rgb
  else if (vil_pixel_format(image) == VIL_RGB_BYTE) {
    vcl_cerr << __FILE__ ": vepl_threshold() cannot be implemented for colour images\n";
    return 0;
  }

  // float
  else if (vil_pixel_format(image) == VIL_FLOAT) {
    vil_memory_image_of<float> mem(image); // load in memory to pass to filter
    vil_memory_image_of<float> out(image);
    vipl_threshold<vil_image,vil_image,float,float,vipl_trivial_pixeliter> op(threshold, below, above);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  // double
  else if (vil_pixel_format(image) == VIL_DOUBLE) {
    vil_memory_image_of<double> mem(image); // load in memory to pass to filter
    vil_memory_image_of<double> out(image);
    vipl_threshold<vil_image,vil_image,double,double,vipl_trivial_pixeliter> op(threshold, below, above);
    op.put_in_data_ptr(&mem);
    op.put_out_data_ptr(&out);
    op.filter();
    return out;
  }

  //
  else {
    vcl_cerr << __FILE__ ": vepl_threshold() not implemented for " << image << vcl_endl;
    return 0;
  }
}

