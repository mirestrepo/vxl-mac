// This is brl/bseg/brip/brip_float_ops.h
#ifndef brip_float_ops_h_
#define brip_float_ops_h_

//-----------------------------------------------------------------------------
//:
// \file
// \author J.L. Mundy
// \brief operations on memory_image_of<float> operands
//
// Thse methods are similar to the VanDuc gevd_float_ops methods. However,
// they use vil1_image_of<float> buffers rather than the old bufferxy
// structure. The purpose is to provide efficient foundational
// segmentation routines. They are not meant to be generic.
//
// \verbatim
//   Initial version February 15, 2003
// \endverbatim
//
//-----------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vbl/vbl_array_2d.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_memory_image_of.h>
#include <vil1/vil1_rgb.h>

class brip_float_ops
{
 public:
  ~brip_float_ops() {}

  //: convolves with the specified kernel
  static vil1_memory_image_of<float>
    convolve(vil1_memory_image_of<float> const & input,
             vbl_array_2d<float> const & kernel);

  //: convolves with the specified kernel
  static vil1_memory_image_of<float>
    gaussian(vil1_memory_image_of<float> const & input, float sigma);

  //: non-maximum suppression on a NxN neighborhood, with sub-pixel location
  static void non_maximum_suppression(vil1_memory_image_of<float> const & input,
                                      const int n,
                                      const float thresh,
                                      vcl_vector<float>& x_pos,
                                      vcl_vector<float>& y_pos,
                                      vcl_vector<float>& value);

  //:downsamples the input using the Bert-Adelson algorithm
  static vil1_memory_image_of<float> 
  half_resolution(vil1_memory_image_of<float> const & input,
                  float filter_coef=0.359375);

#if 0
  //: interpolates the input using the Bert-Adelson algorithm
  static vil1_memory_image_of<float>
    double_resolution(vil1_memory_image_of<float> const & input,
                      float filter_coef=0.359375);
#endif

  //: subtracts image_1 from image_2
  static vil1_memory_image_of<float>
    difference(vil1_memory_image_of<float> const & image_1,
               vil1_memory_image_of<float> const & image_2);

  //: The gradient using a 3x3 kernel
  static void gradient_3x3(vil1_memory_image_of<float> const & input,
                           vil1_memory_image_of<float>& grad_x,
                           vil1_memory_image_of<float>& grad_y);

  static void hessian_3x3(vil1_memory_image_of<float> const & input,
                          vil1_memory_image_of<float>& Ixx,
                          vil1_memory_image_of<float>& Ixy,
                          vil1_memory_image_of<float>& Iyy);

  static vil1_memory_image_of<float>
  beaudet(vil1_memory_image_of<float> const & Ixx,
          vil1_memory_image_of<float> const & Ixy,
          vil1_memory_image_of<float> const & Iyy);


  //: IxIx.transpose gradient matrix elements (N = 2n+1)
  static void grad_matrix_NxN(vil1_memory_image_of<float> const & input,
                              const int n,
                              vil1_memory_image_of<float>& IxIx,
                              vil1_memory_image_of<float>& IxIy,
                              vil1_memory_image_of<float>& IyIy);

  //: Computes the Harris corner measure
  static vil1_memory_image_of<float>
  harris(vil1_memory_image_of<float> const & IxIx,
         vil1_memory_image_of<float> const & IxIy,
         vil1_memory_image_of<float> const & IyIy,
         double scale=0.04);


  //: computes the conditioning of the 2n+1 x 2n+1 gradient neigborhood
  static vil1_memory_image_of<float>
    sqrt_grad_singular_values(vil1_memory_image_of<float> & input, int n);

  //: computes Lucas-Kanade optical flow on a 2n+1 neighborhood
  static void Lucas_KanadeMotion(vil1_memory_image_of<float> & current_frame,
                                 vil1_memory_image_of<float> & previous_frame,
                                 int n, double thresh,
                                 vil1_memory_image_of<float>& vx,
                                 vil1_memory_image_of<float>& vy);

  //: fills a border of width w on left and right of image with value
  static void fill_x_border(vil1_memory_image_of<float> & image,
                            int w, float value);

  //: fills a border of width h on top and bottom of image with value
  static void fill_y_border(vil1_memory_image_of<float> & image,
                            int h, float value);

  //: converts a float image to a byte value range
  static vil1_memory_image_of<unsigned char>
    convert_to_byte(vil1_memory_image_of<float> const & image);

  //: converts a float image to a byte value range within a specified range
  static vil1_memory_image_of<unsigned char>
    convert_to_byte(vil1_memory_image_of<float> const & image,
                    const float min_val, const float max_val);

  //: converts an RGB image to a float image
  static vil1_memory_image_of<unsigned short>
    convert_to_short(vil1_memory_image_of<float> const & image,
                     const float min_val, const float max_val);

  //: converts a vil1_image to a float image
  static vil1_memory_image_of<float>
    convert_to_float(vil1_image const & image);

  //: converts a byte image to a float image
  static vil1_memory_image_of<float>
    convert_to_float(vil1_memory_image_of<unsigned char> const & image);

  //: converts an RGB image to a float image
  static vil1_memory_image_of<float>
    convert_to_float(vil1_memory_image_of<vil1_rgb<unsigned char> > const& image);

  //: loads a 2n+1 x 2n+1 convolution kernel (see .cxx for file format)
  static vbl_array_2d<float> load_kernel(vcl_string const & file);

 private:

  //:find if the center pixel of a neighborhood is the maximum value
  static bool local_maximum(vbl_array_2d<float> const & nighborhood,
                     int n, float& value);

  //:find the sub-pixel offset to the maximum using a 3x3 quad interpolation
  static void interpolate_center(vbl_array_2d<float> const & neighborhood,
                          float& dx, float& dy);

  //:sub-sample a 1-d array using the Bert-Adelson algorithm
  static void half_resolution_1d(const float* input, int n, 
                                 const float k0, const float k1,
                                 const float k2, float* output);
  brip_float_ops() {}
};

#endif // brip_float_ops_h_
