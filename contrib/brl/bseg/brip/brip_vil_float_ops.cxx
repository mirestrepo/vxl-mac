#include "brip_vil_float_ops.h"
//:
// \file

#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include <vnl/vnl_numeric_traits.h>
#include <vnl/vnl_math.h>
#include <vcl_complex.h>
#include <vnl/algo/vnl_fft_prime_factors.h>
#include <vnl/algo/vnl_svd.h>
#include <vil/vil_pixel_format.h>
#include <vil/vil_transpose.h>
#include <vil/algo/vil_convolve_1d.h>

//------------------------------------------------------------
//:  Convolve with a kernel
//   It's assumed that the kernel is square with odd dimensions
vil_image_view<float>
brip_vil_float_ops::convolve(vil_image_view<float> const & input,
                         vbl_array_2d<float> const & kernel)
{
  int w = input.ni(), h = input.nj();
  int kw = kernel.cols(); // kh = kernel.rows();
  // add a check for kernels that are not equal dimensions of odd size JLM
  int n = (kw-1)/2;
  vil_image_view<float> output;
  output.set_size(w,h);
  for (int y = n; y<(h-n); y++)
    for (int x = n; x<(w-n); x++)
    {
      float accum = 0;
      for (int j = -n; j<=n; j++)
        for (int i = -n; i<=n; i++)
        {
          float x1 = input(x+i,y+j);
          float x2 = kernel[i+n][j+n];
          accum += x1*x2;
        }
      output(x,y)=accum;
    }
  brip_vil_float_ops::fill_x_border(output, n, 0.0f);
  brip_vil_float_ops::fill_y_border(output, n, 0.0f);
  return output;
}

static void fill_1d_array(vil_image_view<float> const & input,
                          const int y, float* output)
{
  int w = input.ni();
  for (int x = 0; x<w; x++)
    output[x] = input(x,y);
}

//: Downsamples the 1-d array by 2 using the Burt-Adelson reduction algorithm.
void brip_vil_float_ops::half_resolution_1d(const float* input, int width,
                                        const float k0, const float k1,
                                        const float k2, float* output)
{
  float w[5];
  int n = 0;
  for (; n<5; n++)
    w[n]=input[n];
  output[0]=k0*w[0]+ 2.0f*(k1*w[1] + k2*w[2]);//reflect at boundary
  for (int x = 1; x<width; x++)
  {
    output[x]=k0*w[2]+ k1*(w[1]+w[3]) + k2*(w[0]+w[4]);
    //shift the window, w, over by two pixels
    w[0] = w[2];       w[1] = w[3];     w[2] = w[4];
    //handle the boundary conditions
    if (x<width-2)
      {w[3] = input[n++]; w[4]= input[n++];}
    else
      {w[3] =w[1]; w[4]= w[0];}
  }
}

//: Downsamples the image by 2 using the Burt-Adelson reduction algorithm.
// Convolution with a 5-point kernel [(0.5-ka)/2, 0.25, ka, 0.25, (0.5-ka)/2]
// ka = 0.6  maximum decorrelation, wavelet for image compression.
// ka = 0.5  linear interpolation,
// ka = 0.4  Gaussian filter
// ka = 0.359375 min aliasing, wider than Gaussian
// The image sizes are related by: output_dimension = (input_dimension +1)/2.
vil_image_view<float>
brip_vil_float_ops::half_resolution(vil_image_view<float> const & input,
                                float filter_coef)
{
  vul_timer t;
  float k0 = filter_coef, k1 = 0.25f*filter_coef, k2 = 0.5f*(0.5f-filter_coef);
  int w = input.ni(), h = input.nj();
  int half_w =(w+1)/2, half_h = (h+1)/2;
  vil_image_view<float> output;
  output.set_size(half_w, half_h);
  //Generate input/output arrays
  int n = 0;
  float* in0 = new float[w];  float* in1 = new float[w];
  float* in2 = new float[w];  float* in3 = new float[w];
  float* in4 = new float[w];

  float* out0 = new float[half_w];  float* out1 = new float[half_w];
  float* out2 = new float[half_w];  float* out3 = new float[half_w];
  float* out4 = new float[half_w];
  //Initialize arrays
  fill_1d_array(input, n++, in0);   fill_1d_array(input, n++, in1);
  fill_1d_array(input, n++, in2);   fill_1d_array(input, n++, in3);
  fill_1d_array(input, n++, in4);

  //downsample initial arrays
  brip_vil_float_ops::half_resolution_1d(in0, half_w, k0, k1, k2, out0);
  brip_vil_float_ops::half_resolution_1d(in1, half_w, k0, k1, k2, out1);
  brip_vil_float_ops::half_resolution_1d(in2, half_w, k0, k1, k2, out2);
  brip_vil_float_ops::half_resolution_1d(in3, half_w, k0, k1, k2, out3);
  brip_vil_float_ops::half_resolution_1d(in4, half_w, k0, k1, k2, out4);
  int x=0, y;
  //do the first output line
  for (;x<half_w;x++)
    output(x,0)= k0*out0[x]+ 2.0f*(k1*out1[x]+k2*out2[x]);
  //normal lines
  for (y=1; y<half_h; y++)
  {
    for (x=0; x<half_w; x++)
      output(x,y) = k0*out2[x]+ k1*(out1[x]+out3[x]) + k2*(out0[x]+out4[x]);
    //shift the neighborhood down two lines
    float* temp0 = out0;
    float* temp1 = out1;
    out0 = out2;  out1 = out3;  out2 = out4;
    out3 = temp0; out4 = temp1;//reflect values
    //test border condition
    if (y<half_h-2)
    {
      //normal processing, so don't reflect
      fill_1d_array(input, n++, in3);
      fill_1d_array(input, n++, in4);
      brip_vil_float_ops::half_resolution_1d(in3, half_w, k0, k1, k2, out3);
      brip_vil_float_ops::half_resolution_1d(in4, half_w, k0, k1, k2, out4);
    }
  }
  delete [] in0;  delete [] in1; delete [] in2;
  delete [] in3;  delete [] in4;
  delete [] out0;  delete [] out1; delete [] out2;
  delete [] out3;  delete [] out4;
  vcl_cout << "\nDownsample a "<< w <<" x " << h << " image in "<< t.real() << " msecs.\n";
  return output;
}

static double brip_vil_gaussian(const double x, const double sigma)
{
  double x_on_sigma = x / sigma;
  return (double)vcl_exp(- x_on_sigma * x_on_sigma / 2);
}

//:generate a 1-d Gaussian kernel  fuzz=0.02 is a good value
static void brip_1d_gaussian_kernel(const double sigma,
                                    const double fuzz,
                                    int& radius,
                                    double*& kernel)
{
  for (radius = 0; brip_vil_gaussian(double(radius), sigma) > fuzz; radius++)
    {;}                                         // find radius

  kernel = new double[2*radius + 1];
  if(!radius)
    {
      kernel[0]=1;
      return;
    }
  for (int i=0; i<=radius; ++i)
    kernel[radius+i] = kernel[radius-i] = brip_vil_gaussian(double(i), sigma);
  double sum = 0;
  for (int i= 0; i <= 2*radius; ++i)
    sum += kernel[i];                           // find integral of weights
  for (int i= 0; i <= 2*radius; ++i)
    kernel[i] /= sum;                           // normalize by integral
}


vil_image_view<float>
brip_vil_float_ops::gaussian(vil_image_view<float> const & input, float sigma)
{
  vil_image_view<float> dest(input.ni(), input.nj());
  int r;
  double* ker;
  brip_1d_gaussian_kernel(sigma, 0.02, r, ker);  
  vil_image_view<float> work(input.ni(), input.nj());
  // filter horizontal
  int ksize = 2*r + 1 ;
  float accum=0;
  vil_convolve_1d(input, work, ker + ksize/2,
                  -ksize/2, r, accum,
                  vil_convolve_ignore_edge,
                  vil_convolve_ignore_edge);
  
  // filter vertical
  vil_image_view<float> work_t = vil_transpose(work);
  vil_image_view<float> dest_t = vil_transpose(dest);
  vil_convolve_1d(work_t, dest_t, ker+ ksize/2,
                  -ksize/2, r, accum, 
                  vil_convolve_ignore_edge,
                  vil_convolve_ignore_edge);
  
  delete ker;
  return dest;
}

//-------------------------------------------------------------------
// Determine if the center of a (2n+1)x(2n+1) neighborhood is a local maximum
//
bool brip_vil_float_ops::
local_maximum(vbl_array_2d<float> const & neighborhood,
              int n, float& value)
{
  bool local_max = true;
  value = 0;
  float center = neighborhood[n][n];
  for (int y = -n; y<=n; y++)
    for (int x = -n; x<=n; x++)
      local_max = local_max&&(neighborhood[y+n][x+n]<=center);
  if (!local_max)
    return false;
  value = center;
  return true;
}

//-------------------------------------------------------------------
// Interpolate the sub-pixel position of a neighborhood using a
// second order expansion on a 3x3 sub-neighborhood. Return the
// offset to the maximum, i.e. x=x0+dx, y = y0+dy. The design is
// similar to the droid counterpoint by fsm, which uses the Beaudet Hessian
//
void brip_vil_float_ops::
interpolate_center(vbl_array_2d<float> const & neighborhood,
                   float& dx, float& dy)
{
  dx = 0; dy=0;
  //extract the neighborhood
  float n_m1_m1 = neighborhood[0][0];
  float n_m1_0 = neighborhood[0][1];
  float n_m1_1 = neighborhood[0][2];
  float n_0_m1 = neighborhood[1][0];
  float n_0_0 = neighborhood[1][1];
  float n_0_1 = neighborhood[1][2];
  float n_1_m1 = neighborhood[2][0];
  float n_1_0 = neighborhood[2][1];
  float n_1_1 = neighborhood[2][2];

  //Compute the 2nd order quadratic coefficients
  //      1/6 * [ -1  0 +1 ]
  // Ix =       [ -1  0 +1 ]
  //            [ -1  0 +1 ]
  float Ix =(-n_m1_m1+n_m1_1-n_0_m1+n_0_1-n_1_m1+n_1_1)/6.0f;
  //      1/6 * [ -1 -1 -1 ]
  // Iy =       [  0  0  0 ]
  //            [ +1 +1 +1 ]
  float Iy =(-n_m1_m1-n_m1_0-n_m1_1+n_1_m1+n_1_0+n_1_1)/6.0f;
  //      1/3 * [ +1 -2 +1 ]
  // Ixx =      [ +1 -2 +1 ]
  //            [ +1 -2 +1 ]
  float Ixx = ((n_m1_m1+n_0_m1+n_1_m1+n_m1_1+n_0_1+n_1_1)
               -2.0f*(n_m1_0+n_0_0+n_1_0))/3.0f;
  //      1/4 * [ +1  0 -1 ]
  // Ixy =      [  0  0  0 ]
  //            [ -1  0 +1 ]
  float Ixy = (n_m1_m1-n_m1_1+n_1_m1+n_1_1)/4.0f;
  //      1/3 * [ +1 +1 +1 ]
  // Iyy =      [ -2 -2 -2 ]
  //            [ +1 +1 +1 ]
  float Iyy = ((n_m1_m1+n_m1_0+n_m1_1+n_1_m1+n_1_0+n_1_1)
               -2.0f*(n_0_m1 + n_0_0 + n_1_0))/3.0f;
  //
  // The next bit is to find the extremum of the fitted surface by setting its
  // partial derivatives to zero. We need to solve the following linear system :
  // Given the fitted surface is
  // I(x,y) = Io + Ix x + Iy y + 1/2 Ixx x^2 + Ixy x y + 1/2 Iyy y^2
  // we solve for the maximum (x,y),
  //
  //  [ Ixx Ixy ] [ dx ] + [ Ix ] = [ 0 ]      (dI/dx = 0)
  //  [ Ixy Iyy ] [ dy ]   [ Iy ]   [ 0 ]      (dI/dy = 0)
  //
  float det = Ixx*Iyy - Ixy*Ixy;
  // det>0 corresponds to a true local extremum otherwise a saddle point
  if (det>0)
  {
    dx = (Iy*Ixy - Ix*Iyy) / det;
    dy = (Ix*Ixy - Iy*Ixx) / det;
    // more than one pixel away
    if (vcl_fabs(dx) > 1.0 || vcl_fabs(dy) > 1.0)
      dx = 0; dy = 0;
  }
}

//---------------------------------------------------------------
// Compute the local maxima of the input on a (2n+1)x(2n+2)
// neighborhood above the given threshold. At each local maximum,
// compute the sub-pixel location, (x_pos, y_pos).
void brip_vil_float_ops::
non_maximum_suppression(vil_image_view<float> const & input,
                        const int n,
                        const float thresh,
                        vcl_vector<float>& x_pos,
                        vcl_vector<float>& y_pos,
                        vcl_vector<float>& value)
{
  vul_timer t;
  int N = 2*n+1;
  int w = input.ni(), h = input.nj();
  x_pos.clear();  x_pos.clear();   value.clear();
  vbl_array_2d<float> neighborhood(N,N);
  for (int y =n; y<h-n; y++)
    for (int x = n; x<w-n; x++)
    {
      //If the center is not above threshold then there is
      //no hope
      if (input(x,y)<thresh)
        continue;
      //Fill the neighborhood
      for (int i = -n; i<=n; i++)
        for (int j = -n; j<=n; j++)
          neighborhood.put(j+n,i+n,input(x+i, y+j));
      //Check if the center is a local maximum
      float dx, dy, max_v;
      if (brip_vil_float_ops::local_maximum(neighborhood, n, max_v))
      {
        //if so sub-pixel interpolate (3x3) and output results
        brip_vil_float_ops::interpolate_center(neighborhood, dx, dy);
        x_pos.push_back(x+dx);
        y_pos.push_back(y+dy);
        value.push_back(max_v);
      }
    }
  vcl_cout << "\nCompute non-maximum suppression on a "<< w <<" x " << h << " image in "<< t.real() << " msecs.\n";
}

// -----------------------------------------------------------------
// Subtract image_1 from image_2.
// Will not operate unless the two input images are the same dimensions
//
vil_image_view<float>
brip_vil_float_ops::difference(vil_image_view<float> const & image_1,
                           vil_image_view<float> const & image_2)
{
  int w1 = image_1.ni(), h1 = image_1.nj();
  int w2 = image_2.ni(), h2 = image_2.nj();
  vil_image_view<float> temp(w1, h1);
  if (w1!=w2||h1!=h2)
  {
    vcl_cout << "In brip_vil_float_ops::difference(..) - images are not the same dimensions\n";
    return temp;
  }
  vil_image_view<float> out;
  out.set_size(w1, h1);
  for (int y = 0; y<h1; y++)
    for (int x = 0; x<w1; x++)
      out(x,y) = image_2(x,y)-image_1(x,y);
  return out;
}

vil_image_view<float>
brip_vil_float_ops::abs_clip_to_level(vil_image_view<float> const & image,
                                  const float thresh, const float level)
{
  vil_image_view<float> out;
  int w = image.ni(), h = image.nj();
  out.set_size(w, h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      if (vcl_fabs(image(x,y))>thresh)
        out(x,y) = level;
      else
        out(x,y) = image(x,y);
    }
  return out;
}

//----------------------------------------------------------------
// Compute the gradient of the input, use a 3x3 mask
//
//         1  |-1  0  1|         1  |-1 -1 -1|
//   Ix = --- |-1  0  1|   Iy = --- | 0  0  0|
//         6  |-1  0  1|         6  | 1  1  1|
//
// Larger masks are computed by pre-convolving with a Gaussian
//
void brip_vil_float_ops::gradient_3x3(vil_image_view<float> const & input,
                                  vil_image_view<float>& grad_x,
                                  vil_image_view<float>& grad_y)
{
  vul_timer t;
  int w = input.ni(), h = input.nj();
  float scale = 1.0f/6.0f;
  for (int y = 1; y<h-1; y++)
    for (int x = 1; x<w-1; x++)
    {
      float gx = input(x+1,y-1)+input(x+1,y)+ input(x+1,y-1)
        -input(x-1,y-1) -input(x-1,y) -input(x-1,y-1);
      float gy = input(x+1,y+1)+input(x,y+1)+ input(x-1,y+1)
        -input(x+1,y-1) -input(x,y-1) -input(x-1,y-1);
      grad_x(x,y) = scale*gx;
      grad_y(x,y) = scale*gy;
    }
  brip_vil_float_ops::fill_x_border(grad_x, 1, 0.0f);
  brip_vil_float_ops::fill_y_border(grad_x, 1, 0.0f);
  brip_vil_float_ops::fill_x_border(grad_y, 1, 0.0f);
  brip_vil_float_ops::fill_y_border(grad_y, 1, 0.0f);
  //  vcl_cout << "\nCompute Gradient in " << t.real() << " msecs.\n";
}

//----------------------------------------------------------------
// Compute the Hessian of the input, use a 3x3 mask
//
//          1 | 1  -2  1|          1 |  1  1  1|         1  | 1  0 -1|
//   Ixx = ---| 1  -2  1|   Iyy = ---| -2 -2 -2|  Ixy = --- | 0  0  0|
//          3 | 1  -2  1|          3 |  1  1  1|         4  |-1  0  1|
//
// Larger masks are computed by pre-convolving with a Gaussian
//
void brip_vil_float_ops::hessian_3x3(vil_image_view<float> const & input,
                                 vil_image_view<float>& Ixx,
                                 vil_image_view<float>& Ixy,
                                 vil_image_view<float>& Iyy)
{
  vul_timer t;
  int w = input.ni(), h = input.nj();
  for (int y = 1; y<h-1; y++)
    for (int x = 1; x<w-1; x++)
    {
      float xx = input(x-1,y-1)+input(x-1,y)+input(x+1,y)+
                 input(x+1,y-1)+input(x+1,y)+input(x+1,y+1)-
                 2.0f*(input(x,y-1)+input(x,y)+input(x,y+1));

      float xy = (input(x-1,y-1)+input(x+1,y+1))-
                 (input(x-1,y+1)+input(x+1,y-1));

      float yy = input(x-1,y-1)+input(x,y-1)+input(x+1,y-1)+
                 input(x-1,y+1)+input(x,y+1)+input(x+1,y+1)-
                 2.0f*(input(x-1,y)+input(x,y)+input(x+1,y));

      Ixx(x,y) = xx/3.0f;
      Ixy(x,y) = xy/4.0f;
      Iyy(x,y) = yy/3.0f;
    }
  brip_vil_float_ops::fill_x_border(Ixx, 1, 0.0f);
  brip_vil_float_ops::fill_y_border(Ixx, 1, 0.0f);
  brip_vil_float_ops::fill_x_border(Ixy, 1, 0.0f);
  brip_vil_float_ops::fill_y_border(Ixy, 1, 0.0f);
  brip_vil_float_ops::fill_x_border(Iyy, 1, 0.0f);
  brip_vil_float_ops::fill_y_border(Iyy, 1, 0.0f);
  vcl_cout << "\nCompute a hessian matrix "<< w <<" x " << h << " image in "<< t.real() << " msecs.\n";
}

vil_image_view<float>
brip_vil_float_ops::beaudet(vil_image_view<float> const & Ixx,
                        vil_image_view<float> const & Ixy,
                        vil_image_view<float> const & Iyy
                       )
{
  int w = Ixx.ni(), h = Ixx.nj();
  vil_image_view<float> output;
  output.set_size(w, h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      float xx = Ixx(x,y), xy = Ixy(x,y), yy = Iyy(x,y);

      //compute eigenvalues for experimentation
      float det = xx*yy-xy*xy;
      float tr = xx+yy;
      float arg = tr*tr-4.f*det, lambda0 = 0, lambda1=0;
      if (arg>0)
      {
        lambda0 = tr+vcl_sqrt(arg);
        lambda1 = tr-vcl_sqrt(arg);
      }
      output(x,y) = lambda0*lambda1; //just det for now
    }
  return output;
}

//----------------------------------------------------------------
//   t
//IxIx gradient matrix elements
// That is,
//                        _                           _
//                       | (dI/dx)^2    (dI/dx)(dI/dy) |
//                       |                             |
//  A = Sum(neighborhood)|                             |
//                       |(dI/dx)(dI/dy)   (dI/dx)^2   |
//                       |_                           _|
//
// over a a 2n+1 x 2n+1 neigborhood
//
void
brip_vil_float_ops::grad_matrix_NxN(vil_image_view<float> const & input,
                                const int n,
                                vil_image_view<float>& IxIx,
                                vil_image_view<float>& IxIy,
                                vil_image_view<float>& IyIy)
{
  int w = input.ni(), h = input.nj();
  int N = (2*n+1)*(2*n+1);
  vil_image_view<float> grad_x, grad_y, output;
  grad_x.set_size(w,h);
  grad_y.set_size(w,h);
  output.set_size(w,h);
  brip_vil_float_ops::gradient_3x3(input, grad_x, grad_y);
  vul_timer t;
  for (int y = n; y<h-n;y++)
    for (int x = n; x<w-n;x++)
    {
      float xx=0, xy=0, yy=0;
      for (int i = -n; i<=n; i++)
        for (int j = -n; j<=n; j++)
        {
          float gx = grad_x(x+i, y+j), gy = grad_y(x+i, y+j);
          xx += gx*gx;
          xy += gx*gy;
          yy += gy*gy;
        }
      IxIx(x,y) = xx/N;
      IxIy(x,y) = xy/N;
      IyIy(x,y) = yy/N;
    }
  brip_vil_float_ops::fill_x_border(IxIx, n, 0.0f);
  brip_vil_float_ops::fill_y_border(IxIx, n, 0.0f);
  brip_vil_float_ops::fill_x_border(IxIy, n, 0.0f);
  brip_vil_float_ops::fill_y_border(IxIy, n, 0.0f);
  brip_vil_float_ops::fill_x_border(IyIy, n, 0.0f);
  brip_vil_float_ops::fill_y_border(IyIy, n, 0.0f);
  vcl_cout << "\nCompute a gradient matrix "<< w <<" x " << h << " image in "<< t.real() << " msecs.\n";
}

vil_image_view<float>
brip_vil_float_ops::harris(vil_image_view<float> const & IxIx,
                       vil_image_view<float> const & IxIy,
                       vil_image_view<float> const & IyIy,
                       const double scale)

{
  int w = IxIx.ni(), h = IxIx.nj();
  float norm = 1e-3f; // Scale the output to values in the 10->1000 range
  vil_image_view<float> output;
  output.set_size(w, h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      float xx = IxIx(x,y), xy = IxIy(x,y), yy = IyIy(x,y);
      float det = xx*yy-xy*xy, trace = xx+yy;
      output(x,y) = float(det - scale*trace*trace)*norm;
    }
  return output;
}

//----------------------------------------------------------------
// Compute the sqrt of the product of the eigenvalues of the
// gradient matrix over a 2n+1 x 2n+1 neigborhood
// That is,
//                        _                           _
//                       | (dI/dx)^2    (dI/dx)(dI/dy) |
//                       |                             |
//  A = Sum(neighborhood)|                             |
//                       |(dI/dx)(dI/dy)   (dI/dx)^2   |
//                       |_                           _|
//
//  The output image is sqrt(lamba_1*lambda_2) where lambda_i are the
//  eigenvalues
//
vil_image_view<float>
brip_vil_float_ops::sqrt_grad_singular_values(vil_image_view<float> & input,
                                          int n)
{
  int N = (2*n+1)*(2*n+1);
  int w = input.ni(), h = input.nj();
  vil_image_view<float> grad_x, grad_y, output;
  grad_x.set_size(w,h);
  grad_y.set_size(w,h);
  output.set_size(w,h);
  brip_vil_float_ops::gradient_3x3(input, grad_x, grad_y);
  vul_timer t;
  for (int y = n; y<h-n;y++)
    for (int x = n; x<w-n;x++)
    {
      float IxIx=0, IxIy=0, IyIy=0;
      for (int i = -n; i<=n; i++)
        for (int j = -n; j<=n; j++)
        {
          float gx = grad_x(x+i, y+j), gy = grad_y(x+i, y+j);
          IxIx += gx*gx;
          IxIy += gx*gy;
          IyIy += gy*gy;
        }
      float det = (IxIx*IyIy-IxIy*IxIy)/N;
      output(x,y)=vcl_sqrt(vcl_fabs(det));
    }
  brip_vil_float_ops::fill_x_border(output, n, 0.0f);
  brip_vil_float_ops::fill_y_border(output, n, 0.0f);
  vcl_cout << "\nCompute sqrt(sigma0*sigma1) in" << t.real() << " msecs.\n";
  return output;
}

//---------------------------------------------------------------------
// Lucas-Kanade motion vectors:  Solve for the motion vectors over a
// (2n+1)x(2n+1) neighborhood. The time derivative of intensity is computed
// from the previous_frame. The threshold eliminates small values of
// the product of the time derivative and the motion matrix eigenvalues,
// i.e, |lambda_1*lambda_2*dI/dt|<thresh.  Thus motion is only reported when
// the solution is well-conditioned.
//
void
brip_vil_float_ops::Lucas_KanadeMotion(vil_image_view<float> & current_frame,
                                   vil_image_view<float> & previous_frame,
                                   int n, double thresh,
                                   vil_image_view<float>& vx,
                                   vil_image_view<float>& vy)
{
  int N = (2*n+1)*(2*n+1);
  int w = current_frame.ni(), h = current_frame.nj();
  vil_image_view<float> grad_x, grad_y, diff;
  grad_x.set_size(w,h);
  grad_y.set_size(w,h);
  //compute the gradient vector and the time derivative
  brip_vil_float_ops::gradient_3x3(current_frame, grad_x, grad_y);
  diff = brip_vil_float_ops::difference(previous_frame, current_frame);
  vul_timer t;
  //sum the motion terms over the (2n+1)x(2n+1) neighborhood.
  for (int y = n; y<h-n;y++)
    for (int x = n; x<w-n;x++)
    {
      float IxIx=0, IxIy=0, IyIy=0, IxIt=0, IyIt=0;
      for (int i = -n; i<=n; i++)
        for (int j = -n; j<=n; j++)
        {
          float gx = grad_x(x+i, y+j), gy = grad_y(x+i, y+j);
          float dt = diff(x+i, y+j);
          IxIx += gx*gx;
          IxIy += gx*gy;
          IyIy += gy*gy;
          IxIt += gx*dt;
          IyIt += gy*dt;
        }
      //Divide by the number of pixels in the neighborhood
      IxIx/=N;  IxIy/=N; IyIy/=N; IxIt/=N; IyIt/=N;
      float det = float(IxIx*IyIy-IxIy*IxIy);
      //Eliminate small motion factors
      float dif = diff(x,y);
      float motion_factor = vcl_fabs(det*dif);
      if (motion_factor<thresh)
      {
        vx(x,y) = 0.0f;
        vy(x,y) = 0.0f;
        continue;
      }
      //solve for the motion vector
      vx(x,y) = (IyIy*IxIt-IxIy*IyIt)/det;
      vy(x,y) = (-IxIy*IxIt + IxIx*IyIt)/det;
    }
  brip_vil_float_ops::fill_x_border(vx, n, 0.0f);
  brip_vil_float_ops::fill_y_border(vx, n, 0.0f);
  brip_vil_float_ops::fill_x_border(vy, n, 0.0f);
  brip_vil_float_ops::fill_y_border(vy, n, 0.0f);
  vcl_cout << "\nCompute Lucas-Kanade in " << t.real() << " msecs.\n";
}

void brip_vil_float_ops::fill_x_border(vil_image_view<float> & image,
                                   int w, float value)
{
  int width = image.ni(), height = image.nj();
  if (2*w>width)
  {
    vcl_cout << "In brip_vil_float_ops::fill_x_border(..) - 2xborder exceeds image width\n";
    return;
  }
  for (int y = 0; y<height; y++)
    for (int x = 0; x<w; x++)
      image(x, y) = value;

  for (int y = 0; y<height; y++)
    for (int x = width-w; x<width; x++)
      image(x, y) = value;
}

void brip_vil_float_ops::fill_y_border(vil_image_view<float> & image,
                                   int h, float value)
{
  int width = image.ni(), height = image.nj();
  if (2*h>height)
  {
    vcl_cout << "In brip_vil_float_ops::fill_y_border(..) - 2xborder exceeds image height\n";
    return;
  }
  for (int y = 0; y<h; y++)
    for (int x = 0; x<width; x++)
      image(x, y) = value;

  for (int y = height-h; y<height; y++)
    for (int x = 0; x<width; x++)
      image(x, y) = value;
}

vil_image_view<unsigned char>
brip_vil_float_ops::convert_to_byte(vil_image_view<float> const & image)
{
  //determine the max min values
  float min_val = vnl_numeric_traits<float>::maxval;
  float max_val = -min_val;
  int w = image.ni(), h = image.nj();
  vil_image_view<unsigned char> output;
  output.set_size(w,h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      min_val = vnl_math_min(min_val, image(x,y));
      max_val = vnl_math_max(max_val, image(x,y));
    }
  float range = max_val-min_val;
  if (range == 0.f)
    range = 1.f;
  else
    range = 255.f/range;
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      float v = (image(x,y)-min_val)*range;
      output(x,y) = (unsigned char)v;
    }
  return output;
}

//------------------------------------------------------------
// Convert the range between min_val and max_val to 255
vil_image_view<unsigned char>
brip_vil_float_ops::convert_to_byte(vil_image_view<float> const & image,
                                const float min_val, const float max_val)
{
  int w = image.ni(), h = image.nj();
  vil_image_view<unsigned char> output;
  output.set_size(w,h);
  float range = max_val-min_val;
  if (range == 0.f)
    range = 1.f;
  else
    range = 255.f/range;
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      float v = (image(x,y)-min_val)*range;
      if (v>255)
        v=255;
      if (v<0)
        v=0;
      output(x,y) = (unsigned char)v;
    }
  return output;
}

vil_image_view<unsigned short>
brip_vil_float_ops::convert_to_short(vil_image_view<float> const & image,
                                 const float min_val, const float max_val)
{
  int w = image.ni(), h = image.nj();
  float max_short = 65355.f;
  vil_image_view<unsigned short> output;
  output.set_size(w,h);
  float range = max_val-min_val;
  if (!range)
    range = 1;
  else
    range = max_short/range;
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      float v = (image(x,y)-min_val)*range;
      if (v>max_short)
        v=max_short;
      if (v<0)
        v=0;
      output(x,y) = (unsigned short)v;
    }
  return output;
}

vil_image_view<float>
brip_vil_float_ops::convert_to_float(vil_image_view<unsigned char> const & image)
{
  vil_image_view<float> output;
  int w = image.ni(), h = image.nj();
  output.set_size(w,h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
      output(x,y) = (float)image(x,y);
  return output;
}

vil_image_view<float>
brip_vil_float_ops::convert_to_float(vil_image_view<vil_rgb<vxl_byte> > const & image)
{
  vil_image_view<float> output;
  int w = image.ni(), h = image.nj();
  output.set_size(w,h);
  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
    {
      vil_rgb<vxl_byte> rgb = image(x,y);
      output(x,y) = (float)rgb.grey();
    }
  return output;
}

static void rgb_to_ihs(vil_rgb<vxl_byte> const & rgb,
                       float& i, float& h, float& s)
{
  // Reference: figure 13.36, page 595 of Foley & van Dam
  float r = rgb.R(), g = rgb.G(), b = rgb.B();
  i = rgb.grey();
  float maxval;
  float minval;
  float delta;

  maxval = vnl_math_max(r, vnl_math_max(g, b));
  minval = vnl_math_min(r, vnl_math_min(g, b));

  //lightness
  float la = (maxval + minval) / 2.0;
  // Achromatic case, intensity is grey or near black or white
  if (maxval == minval||i<20||i>235)
  {
    s = -1.0;
    h = 0.0;
  }
  else//the chromatic case
  {
    // Calculate the saturation.
    if (la <= 127)
    {
      s = (255.0*(maxval - minval))/(maxval + minval);
    }
    else
    {
      s = (255.0*(maxval - minval)) / (512 - maxval - minval);
      if (s<0)
        s=0;
      if (s>255)
        s=255;
    }

    // Calculate the hue.
    delta = maxval - minval;
    if (r == maxval)
    {
      // The resulting color is between yellow and magenta.
      h = (60*(g - b))/ delta;
    }
    else if (g == maxval)
    {
      // The resulting color is between cyan and yellow.
      h = 120.0 + (60*(b - r))/delta;
    }
    else
    {
      // The resulting color is between magenta and cyan.
      h = 240.0 + (60*(r - g))/delta;
    }
    // Be sure 0.0 <= hue <= 360.0
    if (h < 0.0)
      h += 360;
    if (h > 360.0)
      h -= 360.0;
  }
}


void brip_vil_float_ops::
convert_to_IHS(vil_image_view<vil_rgb<vxl_byte> >const& image,
               vil_image_view<float>& I,
               vil_image_view<float>& H,
               vil_image_view<float>& S)
{
  int w = image.ni(), h = image.nj();
  I.set_size(w,h);
  H.set_size(w,h);
  S.set_size(w,h);
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++)
    {
      float in, hue, sat;
      rgb_to_ihs(image(c,r), in, hue, sat);
      I(c,r) = in;
      H(c,r) = hue;
      S(c,r) = sat;
    }
}

#if 0
void brip_vil_float_ops::
  display_IHS_as_RGB(vil_image_view<float> const& I,
                     vil_image_view<float> const& H,
                     vil_image_view<float> const& S,
                     vil_image_view<vil_rgb<vxl_byte> >& image)
{
  int w = I.ni(), h = I.nj();
  image.set_size(w,h);
  float s = 255.0f/360.0f;
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++)
    {
      float in, hue, sat;
      in = I(c,r);
      hue = H(c,r);
      sat = S(c,r);
      if (in<0)
        in = 0;
      if (sat<0)
        sat = 0;
      if (hue<0)
        hue = 0;
      if (in>255)
        in = 255;
      hue *=s;
      if (hue>255)
        hue = 255;
      if (sat>255)
        sat = 255;
      unsigned char vi = (unsigned char)in, vh = (unsigned char)hue,
        vs = (unsigned char)sat;
      vil_rgb<vxl_byte> v(vi, vh, vs);
      image(c,r)=v;
    }
}
#endif

//: map so that intensity is proportional to saturation and hue is color
void brip_vil_float_ops::
display_IHS_as_RGB(vil_image_view<float> const& I,
                   vil_image_view<float> const& H,
                   vil_image_view<float> const& S,
                   vil_image_view<vil_rgb<vxl_byte> >& image)
{
  int w = I.ni(), h = I.nj();
  image.set_size(w,h);

  float deg_to_rad = vnl_math::pi/180.0f;
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++)
    {
      float hue, sat;
      hue = H(c,r);
      sat = 2.0*S(c,r);
      if (sat<0)
        sat = 0;
      if (sat>255)
        sat = 255;
      float ang = deg_to_rad*hue;
      float cs = vcl_cos(ang), si = vcl_fabs(vcl_sin(ang));
      float red,green,blue;
      green = si*sat;
      if (cs>=0)
      {
        red = cs*sat;
        blue = 0;
      }
      else
      {
        red = 0;
        blue = sat*(-cs);
      }
      unsigned char rc = (unsigned char)red,
        gc = (unsigned char)green, bc = (unsigned char)blue;
      image(c,r)= vil_rgb<vxl_byte>(rc, gc, bc);
    }
}

vil_image_view<float>
brip_vil_float_ops::convert_to_float(vil_image_resource const & image)
{
  vil_image_view<float> fimg;
  if (vil_pixel_format_num_components(image.pixel_format())==1)
  {
    vil_image_view<unsigned char> temp=image.get_view();
    fimg = brip_vil_float_ops::convert_to_float(temp);
  }
  else if (vil_pixel_format_num_components(image.pixel_format())==3)
  {
    vil_image_view<vil_rgb<vxl_byte> > temp= image.get_view();
    fimg = brip_vil_float_ops::convert_to_float(temp);
  }
  else
  {
    vcl_cout << "In brip_vil_float_ops::convert_to_float - input not color or grey\n";
    return vil_image_view<float>();
  }
  return fimg;
}

//-----------------------------------------------------------------
// : convert a vil_rgb<vxl_byte> image to an unsigned_char image.
vil_image_view<unsigned char>
brip_vil_float_ops::convert_to_grey(vil_image_resource const& image)
{

  //Check if the image is a float
  if (image.nplanes()==1 &&image.pixel_format()==VIL_PIXEL_FORMAT_FLOAT)
    {
      vil_image_view<float> temp = image.get_view();
      return brip_vil_float_ops::convert_to_byte(temp);
    }

  //Here we assume that the image is an unsigned char
  //In this case we should just return it.
  if (image.nplanes()==1&&image.pixel_format()==VIL_PIXEL_FORMAT_BYTE)
    {
      vil_image_view<unsigned char > temp = image.get_view();
      return temp;
    }

  // the image is color so we should convert it to greyscale
  // Here we assume the color elements are unsigned char.
  if (image.nplanes()==3&&image.pixel_format()==VIL_PIXEL_FORMAT_BYTE)
    {
      vil_image_view<vil_rgb<vxl_byte> > color_image = image.get_view();
      int width = color_image.ni(), height = color_image.nj();      
      // the output image
      vil_image_view<unsigned char> grey_image;
      grey_image.set_size(width, height);
      for (int y = 0; y<height; y++)
        for (int x = 0; x<width; x++)
          grey_image(x,y) = color_image(x,y).grey();
      return grey_image;
    }
  //If we get here then the input is not a type we handle so return a null view
  return vil_image_view<unsigned char>();
}

//--------------------------------------------------------------
// Read a convolution kernel from file
// Assumes a square kernel with odd dimensions, i.e., w,h = 2n+1
// format:
//     n
//     scale
//     k00  k01  ... k02n
//           ...
//     k2n0 k2n1 ... k2n2n
//
vbl_array_2d<float> brip_vil_float_ops::load_kernel(vcl_string const & file)
{
  vcl_ifstream instr(file.c_str(), vcl_ios::in);
  if (!instr)
  {
    vcl_cout << "In brip_vil_float_ops::load_kernel - failed to load kernel\n";
    return vbl_array_2d<float>(0,0);
  }
  int n;
  float scale;
  float v =0;
  instr >> n;
  instr >> scale;
  int N = 2*n+1;
  vbl_array_2d<float> output(N, N);
  for (int y = 0; y<N; y++)
    for (int x = 0; x<N; x++)
    {
      instr >> v;
      output.put(x, y, v/scale);
    }
  vcl_cout << "The Kernel\n";
  for (int y = 0; y<N; y++)
  {
    for (int x = 0; x<N; x++)
      vcl_cout << ' ' <<  output[x][y];
    vcl_cout << '\n';
  }
  return output;
}

static void insert_image(vil_image_view<float> const& image, int col,
                         vnl_matrix<float> & I)
{
  int width = image.ni(), height = image.nj(), row=0;
  for (int y =0; y<height; y++)
    for (int x = 0; x<width; x++, row++)
      I.put(row, col, image(x,y));
}

void brip_vil_float_ops::
basis_images(vcl_vector<vil_image_view<float> > const & input_images,
             vcl_vector<vil_image_view<float> > & basis)
{
  basis.clear();
  int n_images = input_images.size();
  if (!n_images)
  {
    vcl_cout << "In brip_vil_float_ops::basis_images(.) - no input images\n";
    return;
  }
  int width = input_images[0].ni(), height = input_images[0].nj();
  int npix = width*height;

  //Insert the images into matrix I
  vnl_matrix<float> I(npix, n_images, 0.0);
  for (int i = 0; i<n_images; i++)
    insert_image(input_images[i], i, I);

  //Compute the SVD of matrix I
  vcl_cout << "Computing Singular values of a " <<  npix << " by "
           << n_images << " matrix\n";
  vul_timer t;
  vnl_svd<float> svd(I);
  vcl_cout << "SVD Took " << t.real() << " msecs\n"
           << "Eigenvalues:\n";
  for (int i = 0; i<n_images; i++)
    vcl_cout << svd.W(i) << '\n';

  //Extract the Basis images
  int rank = svd.rank();
  if (!rank)
  {
    vcl_cout << "In brip_vil_float_ops::basis_images(.) - I has zero rank\n";
    return;
  }
  vnl_matrix<float> U = svd.U();
  //Output the basis images
  int rows = U.rows();
  for (int k = 0; k<rank; k++)
  {
    vil_image_view<float> out(width, height);
    int x =0, y = 0;
    for (int r = 0; r<rows; r++)
    {
      out(x, y) = U(r,k);
      x++;
      if (x>=width)
      {
        y++;
        x=0;
      }
      if (y>=width)
      {
        vcl_cout << "In brip_vil_float_ops::basis_images(.) - shouldn't happen\n";
        return;
      }
    }
    basis.push_back(out);
  }
}

//: 1d fourier transform
//-------------------------------------------------------------------------
// This computes an in-place complex-to-complex FFT
// x and y are the real and imaginary arrays of 2^m points.
// dir =  1 gives forward transform
// dir = -1 gives reverse transform
//
//   Formula: forward
//                N-1
//                ---
//            1   \          - j k 2 pi n / N
//    X(n) = ---   >   x(k) e                    = forward transform
//            N   /                                n=0..N-1
//                ---
//                k=0
//
//    Formula: reverse
//                N-1
//                ---
//                \          j k 2 pi n / N
//    X(n) =       >   x(k) e                    = forward transform
//                /                                n=0..N-1
//                ---
//                k=0
//
bool brip_vil_float_ops::fft_1d(int dir, int m, double* x, double* y)
{
  long nn,i,i1,j,k,i2,l,l1,l2;
  double c1,c2,tx,ty,t1,t2,u1,u2,z;

  /* Calculate the number of points */
  nn = 1;
  for (i=0;i<m;i++)
      nn *= 2;

  /* Do the bit reversal */
  i2 = nn >> 1;
  j = 0;
  for (i=0;i<nn-1;i++) {
    if (i < j) {
      tx = x[i];
      ty = y[i];
      x[i] = x[j];
      y[i] = y[j];
      x[j] = tx;
      y[j] = ty;
    }
    k = i2;
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  // Compute the FFT
  c1 = -1.0;
  c2 = 0.0;
  l2 = 1;
  for (l=0;l<m;l++) {
    l1 = l2;
    l2 <<= 1;
    u1 = 1.0;
    u2 = 0.0;
    for (j=0;j<l1;j++) {
      for (i=j;i<nn;i+=l2) {
        i1 = i + l1;
        t1 = u1 * x[i1] - u2 * y[i1];
        t2 = u1 * y[i1] + u2 * x[i1];
        x[i1] = x[i] - t1;
        y[i1] = y[i] - t2;
        x[i] += t1;
        y[i] += t2;
      }
      z =  u1 * c1 - u2 * c2;
      u2 = u1 * c2 + u2 * c1;
      u1 = z;
    }
    c2 = vcl_sqrt((1.0 - c1) / 2.0);
    if (dir == 1)
      c2 = -c2;
    c1 = vcl_sqrt((1.0 + c1) / 2.0);
  }

  // Scaling for forward transform
  if (dir == 1) {
    for (i=0;i<nn;i++) {
      x[i] /= (double)nn;
      y[i] /= (double)nn;
    }
  }

  return true;
}

//-------------------------------------------------------------------------
//  Perform a 2D FFT inplace given a complex 2D array
//  The direction dir, 1 for forward, -1 for reverse
//  The size of the array (nx,ny)
//  Return false if there are memory problems or
//  the dimensions are not powers of 2
//
bool brip_vil_float_ops::fft_2d(vnl_matrix<vcl_complex<double> >& c,int nx,int ny,int dir)
{
  int i,j;
  int mx, my;
  double *real,*imag;
  vnl_fft_prime_factors<double> pfx (nx);
  vnl_fft_prime_factors<double> pfy (ny);
  mx = (int)pfx.pqr()[0];
  my = (int)pfy.pqr()[0];
  /* Transform the rows */
  real = new double[nx];
  imag = new double[nx];
  if (real == 0 || imag == 0)
    return false;
  for (j=0;j<ny;j++) {
    for (i=0;i<nx;i++) {
      real[i] = c[j][i].real();
      imag[i] = c[j][i].imag();
    }
    brip_vil_float_ops::fft_1d(dir,mx,real,imag);
    for (i=0;i<nx;i++) {
      vcl_complex<double> v(real[i], imag[i]);
      c[j][i] = v;
    }
  }
  delete [] real;
  delete [] imag;
  /* Transform the columns */
  real = new double[ny];
  imag = new double[ny];
  if (real == 0 || imag == 0)
    return false;
  for (i=0;i<nx;i++) {
    for (j=0;j<ny;j++) {
      real[j] = c[j][i].real();
      imag[j] = c[j][i].imag();
    }
    fft_1d(dir,my,real,imag);
    for (j=0;j<ny;j++) {
      vcl_complex<double> v(real[j], imag[j]);
      c[j][i] =  v;
    }
  }
  delete [] real;
  delete [] imag;
  return true;
}

//: reorder the transform values to sequential frequencies as in conventional Fourier transforms.
//  The transformation is its self-inverse.
void brip_vil_float_ops::
ftt_fourier_2d_reorder(vnl_matrix<vcl_complex<double> > const& F1,
                       vnl_matrix<vcl_complex<double> > & F2)
{
  int rows = F1.rows(), cols = F1.cols();
  int half_rows = rows/2, half_cols = cols/2;
  int ri, ci;
  for (int r = 0; r<rows; r++)
  {
    if (r<half_rows)
      ri = half_rows+r;
    else
      ri = r-half_rows;
    for (int c = 0; c<cols; c++)
    {
      if (c<half_cols)
        ci = half_cols+c;
      else
        ci = c-half_cols;
      F2[ri][ci]=F1[r][c];
    }
  }
}

//:  Compute the fourier transform.
//   If the image dimensions are not a power of 2 then the operation fails.
bool brip_vil_float_ops::
fourier_transform(vil_image_view<float> const & input,
                  vil_image_view<float>& mag,
                  vil_image_view<float>& phase)
{
  int w = input.ni(), h = input.nj();
  vnl_fft_prime_factors<float> pfx (w);
  vnl_fft_prime_factors<float> pfy (h);
  if (!pfx.pqr()[0]||!pfy.pqr()[0])
    return false;
  //fill the fft matrix
  vnl_matrix<vcl_complex<double> > fft_matrix(h, w), fourier_matrix(h,w);
  for (int y = 0; y<h; y++)
    for (int x =0; x<w; x++)
    {
      vcl_complex<double> cv(input(x,y), 0.0);
      fft_matrix.put(y, x, cv);
    }
#ifdef DEBUG
  for (int r = 0; r<h; r++)
    for (int c =0; c<w; c++)
    {
      vcl_complex<double> res = fft_matrix[r][c];
      vcl_cout << res << '\n';
    }
#endif

  brip_vil_float_ops::fft_2d(fft_matrix, w, h, 1);
  brip_vil_float_ops::ftt_fourier_2d_reorder(fft_matrix, fourier_matrix);
  mag.set_size(w,h);
  phase.set_size(w,h);

  //extract magnitude and phase
  for (int r = 0; r<h; r++)
    for (int c = 0; c<w; c++)
    {
      float re = (float)fourier_matrix[r][c].real(), im = (float)fourier_matrix[r][c].imag();
      mag(c,r) = vcl_sqrt(re*re + im*im);
      phase(c,r) = vcl_atan2(im, re);
    }

  return true;
}

bool brip_vil_float_ops::
inverse_fourier_transform(vil_image_view<float> const& mag,
                          vil_image_view<float> const& phase,
                          vil_image_view<float>& output)
{
  int w = mag.ni(), h = mag.nj();
  vnl_matrix<vcl_complex<double> > fft_matrix(h, w), fourier_matrix(h, w);
  for (int y = 0; y<h; y++)
    for (int x =0; x<w; x++)
    {
      float m = mag(x,y);
      float p = phase(x,y);
      vcl_complex<double> cv(m*vcl_cos(p), m*vcl_sin(p));
      fourier_matrix.put(y, x, cv);
    }

  brip_vil_float_ops::ftt_fourier_2d_reorder(fourier_matrix, fft_matrix);
  brip_vil_float_ops::fft_2d(fft_matrix, w, h, -1);

  output.set_size(w,h);

  for (int y = 0; y<h; y++)
    for (int x = 0; x<w; x++)
      output(x,y) = (float)fft_matrix[y][x].real();
  return true;
}

void brip_vil_float_ops::resize(vil_image_view<float> const & input,
                            const int width, const int height,
                            vil_image_view<float>& output)
{
  int w = input.ni(), h = input.nj();
  output.set_size(width, height);
  for (int y = 0; y<height; y++)
    for (int x = 0; x<width; x++)
      if (x<w && y<h)
        output(x,y) = input(x,y);
      else
        output(x,y) = 0;//pad with zeroes
}

//: resize the input to the closest power of two image dimensions
bool brip_vil_float_ops::
resize_to_power_of_two(vil_image_view<float> const & input,
                       vil_image_view<float>& output)
{
  int max_exp = 13; //we wouldn't want to have such large images in memory
  int w = input.ni(), h = input.nj();
  int prodw = 1, prodh = 1;
  //Find power of two width
  int nw, nh;
  for (nw = 1; nw<=max_exp; nw++)
    if (prodw>w)
      break;
    else
      prodw *= 2;
  if (nw==max_exp)
    return false;
  //Find power of two height
  for (nh = 1; nh<=max_exp; nh++)
    if (prodh>h)
      break;
    else
      prodh *= 2;
  if (nh==max_exp)
    return false;
  brip_vil_float_ops::resize(input, prodw, prodh, output);

  return true;
}

//
//: block a periodic signal by suppressing two Gaussian lobes in the frequency domain.
//  The lobes are on the line defined by dir_fx and dir_fy through the
//  dc origin, assumed (0, 0).  The center frequency, f0, is the distance along
//  the line to the center of each blocking lobe (+- f0). radius is the
//  standard deviation of each lobe. Later we can define a "filter" class.
//
float brip_vil_float_ops::gaussian_blocking_filter(const float dir_fx,
                                               const float dir_fy,
                                               const float f0,
                                               const float radius,
                                               const float fx,
                                               const float fy)
{
  // normalize dir_fx and dir_fy
  float mag = vcl_sqrt(dir_fx*dir_fx + dir_fy*dir_fy);
  if (!mag)
    return 0;
  float r2 = 2.f*radius*radius;
  float dx = dir_fx/mag, dy = dir_fy/mag;
  // compute the centers of each lobe
  float fx0p = dx*f0, fy0p = dy*f0;
  float fx0m = -dx*f0, fy0m = -dy*f0;
  // the squared distance of fx, fy from each center
  float d2p = (fx-fx0p)*(fx-fx0p) + (fy-fy0p)*(fy-fy0p);
  float d2m = (fx-fx0m)*(fx-fx0m) + (fy-fy0m)*(fy-fy0m);
  // use the closest lobe
  float d = d2p;
  if (d2m<d2p)
    d = d2m;
  // the gaussian blocking function
  float gb = 1.f-(float)vcl_exp(-d/r2);
  return gb;
}

bool brip_vil_float_ops::
spatial_frequency_filter(vil_image_view<float> const & input,
                         const float dir_fx, const float dir_fy,
                         const float f0, const float radius,
                         const bool output_fourier_mag,
                         vil_image_view<float> & output)
{
  //Compute the fourier transform of the image.
  vil_image_view<float> pow_two, mag, bmag, phase, pow_two_filt;
  brip_vil_float_ops::resize_to_power_of_two(input, pow_two);
  int Nfx = pow_two.ni(), Nfy = pow_two.nj();

  if (!brip_vil_float_ops::fourier_transform(pow_two, mag, phase))
    return false;
  bmag.set_size(Nfx, Nfy);

  //filter the magnitude function
  float Ofx = Nfx*0.5f, Ofy = Nfy*0.5f;
  for (int fy =0; fy<Nfy; fy++)
    for (int fx =0; fx<Nfx; fx++)
    {
      float gb = gaussian_blocking_filter(dir_fx, dir_fy, f0,
                                          radius,
                                          fx-Ofx, fy-Ofy);
      bmag(fx,fy) = mag(fx,fy)*gb;
    }
  if (output_fourier_mag)
  {
    output = bmag;
    return true;
  }
  //Transform back
  pow_two_filt.set_size(Nfx, Nfy);
  brip_vil_float_ops::inverse_fourier_transform(bmag, phase, pow_two_filt);

  //Resize to original input size
  brip_vil_float_ops::resize(pow_two_filt, input.ni(), input.nj(), output);
  return true;
}

//----------------------------------------------------------------------
//: Bi-linear interpolation on the neigborhood below.
//      xr
//   yr 0  x
//      x  x
//
double brip_vil_float_ops::
  bilinear_interpolation(vil_image_view<float> const & input,
                         const double x, const double y)
{
  //check bounds
  int w = input.ni(), h = input.nj();
  //the pixel containing the interpolated point
  int xr = (int)(x+0.5), yr = (int)(y+0.5);
  if (xr<0||xr>w-2)
    return 0;
  if (yr<0||yr>h-2)
    return 0;
  double int00 = input(xr, yr), int10 = input(xr+1,yr);
  double int01 = input(xr, yr+1), int11 = input(xr+1,yr+1);

  double s00 = (1-x+xr)*(1-y+yr)*int00,   s10 = (x-xr)*(1-y+yr)*int10;
  double s01 = (1-x+xr)*(y-yr)*int01, s11 = (x-xr)*(y-yr)*int11;
  return s00+s01+s10+s11;
}

