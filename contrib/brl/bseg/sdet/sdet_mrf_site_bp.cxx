#include "sdet_mrf_site_bp.h"
#include <vcl_cmath.h>
#include <vnl/vnl_numeric_traits.h>
//:
// \file
#include <vcl_iostream.h>
#include <vcl_string.h>
sdet_mrf_site_bp::sdet_mrf_site_bp(unsigned n_labels, float lambda, float truncation_cost)
  : prior_(0), n_labels_(n_labels), n_ngbh_(4),
    obs_label_(0.0f), lambda_(lambda), truncation_cost_(truncation_cost)
{
  msg_.resize(2);
  msg_[0].resize(n_ngbh_);
  msg_[1].resize(n_ngbh_);
  for(unsigned n = 0; n<n_ngbh_; ++n)
    {
      msg_[0][n].resize(n_labels_, 0);
      msg_[1][n].resize(n_labels_, 0);
    }
}
// the mrf data term
float sdet_mrf_site_bp::D(unsigned fp)
{
  float tmp = (static_cast<float>(fp) - obs_label_);
  tmp *= tmp;
  if(tmp<truncation_cost_)
    return lambda_*tmp;
  return lambda_*truncation_cost_;
}

// the sum of prior messages for fp except for q
// normalized by the number of incoming neighbors
float sdet_mrf_site_bp::M(unsigned nq, unsigned fp)
{
  float sum = 0.0f;
  for(unsigned n = 0; n<n_ngbh_; ++n)
    if(n!=nq)
      sum += msg_[prior_][n][fp];
  return sum;
}

void sdet_mrf_site_bp::set_cur_message(unsigned nq, unsigned fp, float msg)
{
  msg_[1-prior_][nq][fp] = static_cast<short>(msg);
}

vcl_vector<float> sdet_mrf_site_bp::prior_message(unsigned nq)
{
	vcl_vector<float> temp;
    for(unsigned i = 0; i< msg_[prior_][nq].size(); i++)
	temp.push_back(msg_[prior_][nq][i]);
	return temp;
}
void sdet_mrf_site_bp::print_prior_messages()
{
  for(unsigned n = 0; n<4; ++n){
    vcl_cout << "Prior message from neighbor " << n << '\n';
    for(unsigned fp = 0; fp<n_labels_; ++fp)
      vcl_cout << msg_[prior_][n][fp] << ' ';
    vcl_cout << "\n\n";
  }
}
void sdet_mrf_site_bp::print_current_messages()
{
  for(unsigned n = 0; n<4; ++n){
    vcl_cout << "Current message from neighbor " << n << '\n';
    for(unsigned fp = 0; fp<n_labels_; ++fp)
      vcl_cout << msg_[1-prior_][n][fp] << ' ';
    vcl_cout << "\n\n";
  }
}

float sdet_mrf_site_bp::b(unsigned fp)
{
  float sum = 0.0f;
  for(unsigned n = 0; n<4; ++n)
    sum += msg_[prior_][n][fp];
  return D(fp) + sum;
}

unsigned sdet_mrf_site_bp::believed_label()
{
  unsigned lmax =0;
  float mmin = vnl_numeric_traits<float>::maxval;
  for(unsigned d = 0; d<n_labels_; ++d)
    if(b(d)<mmin){
      mmin = b(d);
      lmax = d;
    }
  return lmax;
}
float sdet_mrf_site_bp::expected_label()
{
  float sum = 0.0f, sumn = 0.0f, sumd= 0.0f;
  //find bounds on belief
  float mmin = vnl_numeric_traits<float>::maxval;
  float mmax = -vnl_numeric_traits<float>::maxval;
  for(unsigned d = 0; d<n_labels_; ++d){
    float bl = b(d);
    if(bl<mmin)
      mmin = bl;
    if(bl>mmax)
      mmax = bl;
  }
  if(mmax<=mmin)
    return -1.0f;//nonsense value
  float sc = 1.0f/(mmax-mmin);
  for(unsigned d = 0; d<n_labels_; ++d){
    float p = (-b(d)+mmax)*sc;
    sumn += d*p;
    sumd += p;
  }
  if(sumd<=0) return -1;
  return sumn/sumd;
}
void sdet_mrf_site_bp::print_belief_vector()
{
  vcl_cout << "Belief \n";
  for(unsigned fp = 0; fp<n_labels_; ++fp){
    vcl_cout << this->b(fp) << ' ';
  }
  vcl_cout << '\n';
}

void sdet_mrf_site_bp::clear(){
  for(unsigned p = 0; p<=1; ++p)
    for(unsigned n = 0; n<4; ++n)
      for(unsigned l =0; l<n_labels_; ++l)
        msg_[p][n][l] = 0;
}
//set the prior message
void sdet_mrf_site_bp::set_prior_message(unsigned nq,
                                         vcl_vector<float>const& msg)
{
    for(unsigned i = 0; i<msg.size(); ++i)
	  msg_[prior_][nq][i]=static_cast<short>(msg[i]);
}
