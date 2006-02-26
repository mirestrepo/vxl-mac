// This is brl/bseg/sdet/sdet_nonmax_suppression_params.cxx
#include <sdet/sdet_nonmax_suppression_params.h>
//:
// \file
// See sdet_nonmax_suppression_params.h
//
//-----------------------------------------------------------------------------
#include <vcl_sstream.h>
#include <vcl_iostream.h>

//------------------------------------------------------------------------
// Constructors
//

sdet_nonmax_suppression_params::
sdet_nonmax_suppression_params(const sdet_nonmax_suppression_params& nsp)
  : gevd_param_mixin()
{
  InitParams(nsp.sigma_, nsp.thresh_);
}

sdet_nonmax_suppression_params::
sdet_nonmax_suppression_params(const double sigma, const double thresh)
{
  InitParams(sigma, thresh);
}

void sdet_nonmax_suppression_params::InitParams(double sigma, double thresh)
{
  sigma_= sigma;
  thresh_ = thresh;
}

//-----------------------------------------------------------------------------
//
//:   Checks that parameters are within acceptable bounds
//    Note that msg << ends seems to restart the string and erase the
//    previous string. We should only use it as the last call, use
//    vcl_endl otherwise.
bool sdet_nonmax_suppression_params::SanityCheck()
{
  vcl_stringstream msg;
  bool valid = true;

  if (sigma_<0.5)
  {
    msg << "ERROR: smoothing should be effective, >0.5";
    valid = false;
  }
  if (thresh_<0 || thresh_>100)
  {
    msg << "ERROR: percentage threshold should be between 0 and 100";
    valid = false;
  }
  msg << vcl_ends;

  SetErrorMsg(msg.str().c_str());
  return valid;
}

vcl_ostream& operator<< (vcl_ostream& os, const sdet_nonmax_suppression_params& nsp)
{
  return
  os << "sdet_nonmax_suppression_params:\n[---\n"
     << "Gaussian derivative kernel sigma " << nsp.sigma_ << vcl_endl
     << "Gradient threshold in percentage " << nsp.thresh_ << vcl_endl
     << "---]" << vcl_endl;
}
