// This is tbl/vipl/examples/example_gaussian_convolution.cxx

//:
// \file
//  This example program shows a typical use of the gaussian convolution filter
//  on a ubyte image.  The input image (argv[1]) must be ubyte, and in that
//  case its convolved with a gaussian kernel and the result is written to
//  argv[2] which is always a PGM file image.
//  Uses vipl_gaussian_convolution<vnl_matrix<ubyte>,vnl_matrix<ubyte>,ubyte,ubyte>.
//  The conversion between vil1_image and the in-memory vnl_matrix<ubyte>
//  is done explicitly.
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   April 2002
//
#include <vnl/vnl_matrix.h>
#include <vil1/vil1_pixel.h>
#include <vil1/vil1_memory_image_of.h>

#include <vipl/vipl_with_vnl_matrix/accessors/vipl_accessors_vnl_matrix.h>
#include <vipl/vipl_gaussian_convolution.h>

typedef unsigned char ubyte;

typedef vnl_matrix<ubyte> img_type;

// for I/O:
#include <vil1/vil1_load.h>
#include <vil1/vil1_save.h>
#include <vcl_iostream.h>
#include <vcl_cstdlib.h> // for atof()

int
main(int argc, char** argv) {
  if (argc < 3) { vcl_cerr << "Syntax: example_gaussian_convolution file_in file_out [sigma]\n"; return 1; }

  // The input image:
  vil1_image in = vil1_load(argv[1]);
  if (vil1_pixel_format(in) != VIL1_BYTE) { vcl_cerr << "Please use a ubyte image as input\n"; return 2; }

  // The output image:
  vil1_memory_image_of<ubyte> out(in);

  // The image sizes:
  int xs = in.width();
  int ys = in.height();

  // The value of sigma: (default is 5)
  float sigma = (argc < 4) ? 5.0f : (float)vcl_atof(argv[3]);

  img_type src(ys,xs);
  img_type dst(ys,xs);

  // set the input image:
  in.get_section(src.begin(),0,0,xs,ys);

  // The filter:
  vipl_gaussian_convolution<img_type,img_type,ubyte,ubyte VCL_DFL_TMPL_ARG(vipl_trivial_pixeliter)> op(sigma);
  op.put_in_data_ptr(&src);
  op.put_out_data_ptr(&dst);
  op.filter();

  // Write output:
  out.put_section(dst.begin(),0,0,xs,ys);
  vil1_save(out, argv[2], "pnm");
  vcl_cout << "Written image of type PGM to " << argv[2] << vcl_endl;

  return 0;
}
