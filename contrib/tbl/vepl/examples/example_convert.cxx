// .NAME example_convert
// .SECTION Description
//  This example program shows a typical use of the vepl_convert function on
//  any image.  The input image (argv[1]) is converted to a ubyte image
//  and written to argv[2] which is always a PGM file image.
//  When the input image is RGB, its intensity is extracted.
//  To this end an explicit converter from vil_rgb<ubyte> to ubyte is necessary.
//  When it is short, int or float, an implicit mapping is done to 0--255.
//
// .SECTION Author
//   Peter Vanroose, K.U.Leuven, ESAT/PSI, 15 May 2001, from vipl/examples
//
#include <vil/vil_memory_image_of.h>
#include <vil/vil_rgb.h>

#include <vepl/vepl_convert.h> // this one last!

// for I/O:
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vcl_iostream.h>

typedef unsigned char ubyte;
typedef vil_rgb<ubyte> rgbcell;

int
main(int argc, char** argv) {
  if (argc < 3) { vcl_cerr << "Syntax: example_convert file_in file_out\n"; return 1; }

  // The input image:
  vil_image in = vil_load(argv[1]);

  // The output image:
  vil_memory_image_of<ubyte> out_grey(in);
  vil_memory_image_of<rgbcell> out_rgb(in);

  if (in.planes() == 1 && in.components() == 1) { // monochrome
    ubyte dummy = 0;
    if (in.bits_per_component() == 8)
      vcl_cerr<<"Warning: no conversion necessary\n";
    out_grey = vepl_convert(in, dummy);
    vil_save(out_grey, argv[2], "pnm");
    vcl_cout << "vepl_convert()ed grey image to PGM image " << argv[2] << vcl_endl;
  } else if (in.planes() == 1 && in.components() == 3) { // colour (RGB)
    rgbcell dummy = rgbcell();
    vepl_convert(in,dummy);
    vil_save(out_rgb, argv[2], "pnm");
    vcl_cout << "vepl_convert()ed RGB image to PPM image " << argv[2] << vcl_endl;
  }
  else vcl_cerr << "Cannot handle image with "<< in.planes() <<" planes and "<< in.components() <<" components\n";

  return 0;
}
