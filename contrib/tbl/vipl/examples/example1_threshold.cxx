//:
// \file
//  This example program shows a typical use of the vipl_threshold IP class on
//  a ubyte image.  The input image (argv[1]) must be ubyte, and in that
//  case is thresholded (threshold value argv[3], default 127) to argv[2]
//  which is always a PGM file image.
//  Uses vipl_threshold<vil_image,vil_image,ubyte,ubyte>.
//  The input and output images are directly passed to the filter
//  (be it that, except if NO_MEMORY_IMAGE is set, the input is buffered
//   into a vil_memory_image_of<ubyte>, because otherwise get_pixel()
//   would be very slow!)
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   15 nov. 1997
//
// \verbatim
// Modifications:
//   Peter Vanroose, Aug.2000 - adapted to vxl
// \endverbatim
//
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_memory_image_of.h>
#include <vipl/accessors/vipl_accessors_vil_image.h>
#include <vipl/vipl_threshold.h>
#include <vcl_iostream.h>
typedef unsigned char ubyte;

int
main(int argc, char** argv) {
  if (argc < 3) { vcl_cerr << "Syntax: example1_threshold file_in file_out [threshold]\n"; return 1; }

  // The input image:
  vil_image in = vil_load(argv[1]);
  if (!in) return 2;
  vil_image* src = &in;
#ifndef NO_MEMORY_IMAGE // otherwise get_pixel() would be very slow!!
  vil_memory_image_of<ubyte> mem(in);
  if (!mem) { vcl_cerr << "This is not a ubyte image\n"; return 3; }
  src = &mem;
#endif

  // The output image:
  vil_memory_image_of<ubyte> out(in);
  vil_image* dst = &out;

  // The threshold value:
  ubyte threshold = (argc < 4) ? 127 : vcl_atoi(argv[3]);

  // The filter:
  vipl_threshold<vil_image,vil_image,ubyte,ubyte VCL_DFL_TMPL_ARG(vipl_trivial_pixeliter)> op(threshold,0);
  // without third argument, only set below threshold to 0
  op.put_in_data_ptr(src);
  op.put_out_data_ptr(dst);
  op.filter();

  vil_save(out, argv[2], "pnm");
  vcl_cout << "Written thresholded image of type PGM to " << argv[2] << vcl_endl;
  return 0;
}
