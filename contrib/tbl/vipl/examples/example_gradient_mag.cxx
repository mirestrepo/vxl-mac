//:
// \file
//  This example program shows a typical use of an IP filter, namely
//  the gradient magnitude operator on a greyscale image.  The input image
//  (argv[1]) must be a ubyte image, and in that case its gradient is
//  written to argv[2] which is always a PGM file image.
//  Uses vipl_gradient_mag<vbl_array_2d<ubyte>,vbl_array_2d<ubyte>,ubyte,ubyte>.
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   29 may 1998.
//
// \verbatim
// Modifications:
//   Peter Vanroose, Aug.2000 - adapted to vxl
// \endverbatim
//
#include <vbl/vbl_array_2d.h>
#include <vil1/vil1_pixel.h>
#include <vil1/vil1_memory_image_of.h>

#include <vipl/vipl_with_vbl_array_2d/accessors/vipl_accessors_vbl_array_2d.h>
#include <vipl/vipl_gradient_mag.h>

typedef unsigned char ubyte;
typedef vbl_array_2d<ubyte> img_type;

// for I/O:
#include <vil1/vil1_load.h>
#include <vil1/vil1_save.h>
#include <vcl_iostream.h>

int
main(int argc, char** argv) {
  if (argc < 3) { vcl_cerr << "Syntax: example_gradient_mag file_in file_out\n"; return 1; }

  // The input image:
  vil1_image in = vil1_load(argv[1]);
  if (vil1_pixel_format(in) != VIL1_BYTE) { vcl_cerr << "Please use a ubyte image as input\n"; return 2; }

  // The output image:
  vil1_memory_image_of<ubyte> out(in);
  
  // The image sizes:
  int xs = in.width();
  int ys = in.height();
  
  img_type src(xs, ys);
  img_type dst(xs, ys);

  // set the input image:
  in.get_section(src.begin(),0,0,xs,ys);

  // The filter:
  vipl_gradient_mag<img_type,img_type,ubyte,ubyte VCL_DFL_TMPL_ARG(vipl_trivial_pixeliter)> op;
  op.put_in_data_ptr(&src);
  op.put_out_data_ptr(&dst);
  op.filter();

  // Write output:
  out.put_section(dst.begin(),0,0,xs,ys);
  vil1_save(out, argv[2], "pnm");
  vcl_cout << "Written image of type PGM to " << argv[2] << vcl_endl;

  return 0;
}
