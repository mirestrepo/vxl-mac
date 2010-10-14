#include "bvpl_gauss3d_x_kernel_factory.h"
//:
// \file

#include <vcl_algorithm.h>
#include <vcl_cmath.h> // for std::sqrt() and std::exp()
#include <vnl/vnl_math.h>
#include <vnl/vnl_float_3.h>
#include <bsta/bsta_gauss_if3.h>

// Default Constructor
bvpl_gauss3d_x_kernel_factory::bvpl_gauss3d_x_kernel_factory()
{
  sigma1_ = 0.0f;
  sigma2_ = 0.0f;
  sigma3_ = 0.0f;
  angular_resolution_ = 0;
  rotation_axis_ = canonical_rotation_axis_;
  angle_ = 0.0f;
}

//: Constructs a kernel form gaussian spheroid with sigma parameters s1 and s2. i.e. Cov is diagonal with entries s1, s2, s2
bvpl_gauss3d_x_kernel_factory::bvpl_gauss3d_x_kernel_factory(float s1, float s2)
{
  //set variances of kernel
  sigma1_ = s1;
  sigma2_ = s2;
  sigma3_ = s2;
  
  //this skernel is symmetric around main axis
  angular_resolution_=0;
  
  //initialize variables
  angle_ = 0.0f;
  rotation_axis_ = canonical_rotation_axis_;
  parallel_axis_ = canonical_parallel_axis_;
  
  //create the default kernel
  create_canonical();
}

//: Constructs a kernel form gaussian ellipsoid with sigma parameters s1, s2 and s3. i.e. Cov is diagonal with entries s1, s2, s3
bvpl_gauss3d_x_kernel_factory::bvpl_gauss3d_x_kernel_factory(float s1, float s2, float s3)
{
  //set variances of kernel
  sigma1_ = s1;
  sigma2_ = s2;
  sigma3_ = s3;
  
  //this value is a meant as a limit there is not theoretical meaning to it
  angular_resolution_= float(vnl_math::pi/16.0);
  
  //initialize variables
  angle_ = 0.0f;
  rotation_axis_ = canonical_rotation_axis_;
  parallel_axis_ = canonical_parallel_axis_;

  
  //create the default kernel
  create_canonical();
}

static inline float second_power(float x) { return x*x; }
static inline float third_power(float x) { return x*x*x; }

void bvpl_gauss3d_x_kernel_factory::create_canonical()
{
  bsta_gauss_if3 gauss_kernel(vnl_float_3(0,0,0), vnl_float_3(sigma1_*sigma1_, sigma2_*sigma2_, sigma3_*sigma3_));
  
  typedef vgl_point_3d<float> point_3d;
  typedef bvpl_kernel_dispatch dispatch;
  
  //This is the support of the kernel
  int min_x = -int(3.0f*sigma1_+0.01f);
  int max_x =  int(3.0f*sigma1_+0.01f);
  int min_y = -int(2.0f*sigma2_+0.01f);
  int max_y =  int(2.0f*sigma2_+0.01f);
  int min_z = -int(2.0f*sigma3_+0.01f);
  int max_z =  int(2.0f*sigma3_+0.01f);
  
  for (int x=min_x; x<= max_x; x++)
  {
    for (int y= min_y; y<= max_y; y++)
    {
      for (int z= min_z; z<= max_z; z++)
      {
        vnl_float_3 pt(x,y,z);
        canonical_kernel_.push_back(vcl_pair<point_3d,dispatch>(point_3d(float(x),float(y),float(z)), dispatch(gauss_kernel.gradient(pt)[0])));
      }
    }
  }
  
  //set the dimension of the 3-d grid
  max_point_.set(max_x,max_y,max_z);
  min_point_.set(min_x,min_y,min_z);
  
  //set the current kernel
  kernel_ = canonical_kernel_;
  factory_name_ = name();
  
  return;
}

#if 0 // commented out

/******************Batch Methods ***********************/

//: Creates a vector of kernels with azimuthal(\theta) and elevation(\phi) resolution equal to pi/4.
//  This uses spherical coordinates where \theta \in  [0,2\pi) and \phi \in [0,pi/2)
//  This batch method is specific to a kernel with two equal sides. the reason for this is that in current
//  applications there is no preference in direction other that the orientation of the kernel.
//  A batch method for a "scalene" kernel requires rotation around its main axis.
bvpl_kernel_vector_sptr bvpl_gauss3d_x_kernel_factory::create_kernel_vector()
{
  bvpl_kernel_vector_sptr kernels = new bvpl_kernel_vector();
  float theta_res = float(vnl_math::pi_over_4); //azimuth; phi_res = zenith (from the pole)
  vnl_float_3 axis;
  float theta = 0.0f;
  float phi = 0.0f;
  
  //when zenith angle is 0
  axis[0] =0.0f;
  axis[1] =0.0f;
  axis[2] =1.0f;
  this->set_rotation_axis(axis);
  kernels->kernels_.push_back(vcl_make_pair(axis , new bvpl_kernel(this->create())));
  
  //when zenith is pi/4 traverse all hemisphere
  phi = float(vnl_math::pi_over_4);
  for (;theta < 2.0f*float(vnl_math::pi)-1e-5; theta +=theta_res)
  {
    axis[0] = vcl_cos(theta) * vcl_sin(phi);
    axis[1] = vcl_sin(theta) * vcl_sin(phi);
    axis[2] = vcl_cos(phi);
    this->set_rotation_axis(axis);
    kernels->kernels_.push_back(vcl_make_pair(axis , new bvpl_kernel(this->create())));
  }
  
  //when zenith is pi/2 we only traverse half a hemisphere
  phi = float(vnl_math::pi_over_2);
  theta =0.0f;
  for (;theta < float(vnl_math::pi)-1e-5; theta +=theta_res)
  {
    axis[0] = float(vcl_cos(theta) * vcl_sin(phi));
    axis[1] = float(vcl_sin(theta) * vcl_sin(phi));
    axis[2] = float(vcl_cos(phi));
    this->set_rotation_axis(axis);
    kernels->kernels_.push_back(vcl_make_pair(axis , new bvpl_kernel(this->create())));
  }
  
  return kernels;
}

//: Creates a vector of kernels according to given azimuthal and elevation resolution, and with angle of rotation = angular_resolution_
bvpl_kernel_vector_sptr bvpl_gauss3d_x_kernel_factory::create_kernel_vector(float pi, float phi)
{
  //to be implemented
  return 0;
}

//: Creates a vector of kernels  according to given azimuthal, levation, and angular resolution
bvpl_kernel_vector_sptr bvpl_gauss3d_x_kernel_factory::create_kernel_vector(float pi, float phi, float angular_res)
{
  //to be impemented
  return 0;
}

#endif // 0
