// Example: convolution

#include <vcl_cstring.h>
#include <vcl_cmath.h>   // fabs()
#include <vcl_iostream.h>

#include <vbl/vbl_arg.h>

#include <vil/vil_image_impl.h>
#include <vil/vil_image.h>
#include <vil/vil_memory_image_of.h>
#include <vil/vil_save.h>
#include <vil/vil_load.h>
#include <vil/vil_convolve_simple.h>

double mask_diff[1][3] = {
  {-1, 0, 1},
};

double mask_sobel_x[3][3] = {
  {-1, 0, 1},
  {-2, 0, 2},
  {-1, 0, 1}
};

double mask_bar_5[1][5] = {
  {1, 1, 1, 1, 1},
};

double mask_gauss_17[1][17] = {
  {
    0.0000,
    0.0001,
    0.0018,
    0.0151,
    0.0876,
    0.3439,
    0.9132,
    1.6408,
    1.9947,
    1.6408,
    0.9132,
    0.3439,
    0.0876,
    0.0151,
    0.0018,
    0.0001,
    0.0000
  }
};


struct vil_kernel_info {
  char const* name;
  int w;
  int h;
  double* mask;
};

vil_kernel_info kernels[] = {
  {"diff_x",      3, 1, &mask_diff[0][0]},
  {"diff_y",      1, 3, &mask_diff[0][0]},
  {"sobel_x",     3, 3, &mask_sobel_x[0][0]},
  {"bar_x_5",     5, 1, &mask_bar_5[0][0]},
  {"bar_y_5",     1, 5, &mask_bar_5[0][0]},
  {"gauss_x_17", 17, 1, &mask_gauss_17[0][0]},
  {"gauss_y_17", 1, 17, &mask_gauss_17[0][0]},
  {0, 0}
};

int main(int argc, char ** argv)
{
  vbl_arg<vcl_string> a_input_filename(0, "input");
  vbl_arg<vcl_string> a_output_filename(0, "output");
  vbl_arg<vcl_string> a_kernel(0, "kernel (choose from: sobel_x)", "sobel_x");
  vbl_arg_parse(argc, argv);

  // Load from disk into memory "inimg"
  vil_image in = vil_load(a_input_filename().c_str());
  vil_memory_image_of<unsigned char> inimg(in);
  
  // Build kernel in "kernelimg"
  vil_memory_image_of<float> kernelimg(0,0);
  vcl_string kernel(a_kernel());
  for(vil_kernel_info* kp = kernels; kp->name; ++kp)
    if (kernel == kp->name) {
      kernelimg.resize(kp->w, kp->h);
      double* v = kp->mask;
      double power = 0;
      for(int y = 0; y < kp->h; ++y)
	for(int x = 0; x < kp->w; ++x) {
	  power += fabs(*v);
	  kernelimg[y][x] = *v;
	  ++v;
	}
      // Scale to unit power
      power = 1/power;
      for(int y = 0; y < kp->h; ++y)
	for(int x = 0; x < kp->w; ++x)
	  kernelimg[y][x] *= power;
    }
  if (kernelimg.width() == 0) {
    vcl_cerr << "vil_convolve: unknown kernel [" << kernel << "]\n";
    return -1;
  }

  // Make "outimg" -- needs to be big enough to hold the full valid convolution
  vil_memory_image_of<unsigned char> outimg(in.width()  + kernelimg.width(), 
					    in.height() + kernelimg.height());
  outimg.fill(0);
  
  vil_convolve_simple(inimg, kernelimg, (float*)0, outimg);
  
  vil_save(outimg, a_output_filename().c_str(), in.file_format());
}

// save(crop, "t.pgm", "pnm") is implemented as:
//  (1) makes a file image, format "pnm" (i.e. a vil_pnm_
//         dimensions,component type,size etc of "crop"
//  (2) vil_copy(crop, fileimage)
