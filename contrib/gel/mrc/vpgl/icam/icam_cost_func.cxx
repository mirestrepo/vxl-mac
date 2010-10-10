// This is vpgl/icam/icam_cost_func.cxx
//:
// \file

#include "icam_cost_func.h"
#include "icam_sample.h"

#include <vil/vil_math.h>
#include <vnl/vnl_numeric_traits.h>

//: Constructor
icam_cost_func::icam_cost_func( const vil_image_view<float>& source_img,
                                const vil_image_view<float>& dest_img,
                                const icam_depth_transform& dt)
 : vnl_least_squares_function(1,1),
   source_image_(source_img),
   dest_image_(dest_img),
   dt_(dt), n_samples_(0)
{
  unsigned ni = dest_image_.ni()-2, nj = dest_image_.nj()-2;
  dest_samples_.set_size(ni*nj);
  dest_samples_.fill(0.0);
  unsigned index = 0;
  for(unsigned j = 1; j<=nj; ++j)
    for(unsigned i = 1; i<=ni; ++i)
      dest_samples_[index++]=dest_image_(i,j);
#if 0
  vcl_cout << "dest samples \n";

  int cent = dest_samples_.size()/2-ni/2;
  for(int i = -4; i<=4; ++i)
    vcl_cout << dest_samples_[i+cent] << ' ';
  vcl_cout << '\n';
#endif
  use_gradient_ = false;
  vnl_least_squares_function::init(dt_.n_params(), dest_samples_.size());
}


//: The main function.
//: The main function.
//  Given the parameter vector x, compute the vector of residuals fx.
void 
icam_cost_func::f(vnl_vector<double> const& x, vnl_vector<double>& fx)
{
  dt_.set_params(x);
  vnl_vector<double> from_samples, from_mask;
  icam_sample::sample(dest_image_.ni(), dest_image_.nj(), source_image_,
                      dt_, from_samples, from_mask, n_samples_);
  fx = element_product<double>(from_mask,(from_samples - dest_samples_));    
}
double icam_cost_func::error(vnl_vector_fixed<double,3> rodrigues,
                             vgl_vector_3d<double> trans,
                             double min_allowed_overlap)
{
  vnl_vector<double> pr(dt_.n_params());
  vnl_vector<double> res;
  pr[0]=rodrigues[0];   pr[1]=rodrigues[1];   pr[2]=rodrigues[2];
  pr[3]=trans.x();   pr[4]=trans.y();   pr[5]=trans.z(); 
  dt_.set_params(pr);
  vnl_vector<double> from_samples, from_mask;
  icam_sample::sample(dest_image_.ni(), dest_image_.nj(), source_image_,
                      dt_, from_samples, from_mask, n_samples_);
  double mag = 0;
  for(unsigned i = 0; i<from_samples.size(); ++i)
    if(from_mask[i]>0.0)
      mag += vcl_fabs(from_samples[i]-dest_samples_[i]);
  if(this->frac_samples()>min_allowed_overlap){
    mag/= n_samples_;
    return mag;
  }
  return vnl_numeric_traits<double>::maxval;
}

void icam_cost_func::samples(vnl_vector_fixed<double, 3> rodrigues,
               vgl_vector_3d<double> trans,
               vnl_vector<double>& trans_source,
               vnl_vector<double>& mask,
               vnl_vector<double>& dest)
{
  vnl_vector<double> pr(dt_.n_params());
  vnl_vector<double> res;
  pr[0]=rodrigues[0];   pr[1]=rodrigues[1];   pr[2]=rodrigues[2];
  pr[3]=trans.x();   pr[4]=trans.y();   pr[5]=trans.z(); 
  dt_.set_params(pr);
  icam_sample::sample(dest_image_.ni(), dest_image_.nj(), source_image_,
                      dt_, trans_source, mask, n_samples_);
  dest = dest_samples_;
}

vcl_vector<double> icam_cost_func::error(vnl_vector<double> const& x,
                                         unsigned param_index, double pmin,
                                         double pmax, double pinc)
{
  vcl_vector<double> ret;
  vnl_vector<double> pr(x);
  vnl_vector<double> res;
  for(double p = pmin; p<=pmax; p+=pinc)
    {
      pr[param_index] = p;
      this->f(pr, res);
      double mag = res.magnitude();
      if(n_samples_){
        mag/= n_samples_;
        ret.push_back(mag);
      }else{ret.push_back(vnl_numeric_traits<double>::maxval);
      }
    }
  return ret;
}