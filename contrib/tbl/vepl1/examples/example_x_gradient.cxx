//:
// \file
//  This example program shows a typical use of a gradient function on
//  a colour image.  The input image (argv[1]) must be 24 bit (colour), and in
//  that case its X gradient is written to argv[2] which is always a PPM file.
//  Note that this requires operator-() on the vil1_rgb<ubyte> data type.
//  But this indeed produces a \e colour gradient!
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   15 May 2001, from vipl/examples
//
#include <vepl1/vepl1_x_gradient.h>

// for I/O:
#include <vil1/vil1_load.h>
#include <vil1/vil1_save.h>
#include <vcl_iostream.h>

int
main(int argc, char** argv)
{
  if (argc < 3)
  {
    vcl_cerr << "Syntax: example_x_gradient file_in file_out\n";
    return 1;
  }

  // The input image:
  vil1_image in = vil1_load(argv[1]);

  // The filter:
  vil1_image out = vepl1_x_gradient(in);

  // Write output:
  vil1_save(out, argv[2], "pnm");
  vcl_cout << "Written image of type PNM to " << argv[2] << vcl_endl;

  return 0;
}
