// This is mul/pdf1d/pdf1d_exponential_sampler.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author Tim Cootes
// \brief Sampler class for Univariate exponential distributions

#include "pdf1d_exponential_sampler.h"

#include <vcl_cassert.h>
#include <vcl_cmath.h>
#include <pdf1d/pdf1d_exponential.h>

//=======================================================================
// Dflt ctor
//=======================================================================

pdf1d_exponential_sampler::pdf1d_exponential_sampler():
 rng_(9667566ul)
{
}

//=======================================================================
// Destructor
//=======================================================================

pdf1d_exponential_sampler::~pdf1d_exponential_sampler()
{
}


//=======================================================================
// Method: is_a
//=======================================================================

vcl_string pdf1d_exponential_sampler::is_a() const
{
  return vcl_string("pdf1d_exponential_sampler");
}

//=======================================================================
// Method: is_class
//=======================================================================

bool pdf1d_exponential_sampler::is_class(vcl_string const& s) const
{
  return pdf1d_sampler::is_class(s) || s==pdf1d_exponential_sampler::is_a();
}

//=======================================================================
// Method: clone
//=======================================================================

pdf1d_sampler* pdf1d_exponential_sampler::clone() const
{
  return new pdf1d_exponential_sampler(*this);
}
//=======================================================================

void pdf1d_exponential_sampler::reseed(unsigned long seed)
{
  rng_.reseed(seed);
}
//=======================================================================

//: Set model for which this is an instance
// Error check that it is an axis exponential.
void pdf1d_exponential_sampler::set_model(const pdf1d_pdf& model)
{
  assert(model.is_class("pdf1d_exponential"));
  // cannot use dynamic_cast<> without rtti - PVr
  // rtti currently turned off
  pdf1d_sampler::set_model(model);
}

//=======================================================================

double pdf1d_exponential_sampler::sample()
{
  const pdf1d_exponential & exponential = (const pdf1d_exponential &) model();
  double L = exponential.lambda();

  return  -1.0*vcl_log(rng_.drand64(0,1))/L;
}

//: Fill x with samples possibly chosen so as to represent the distribution
//  5 or fewer samples requested, they are spaced out equally.
void pdf1d_exponential_sampler::regular_samples(vnl_vector<double>& x)
{
  int n = x.size();
  const pdf1d_exponential & exponential = (const pdf1d_exponential &) model();
  double L = exponential.lambda();

  // CDF = 1-exp(-Lx)
  // Require CDF(x) = (1+i)/(n+1)
  // Thus exp(-Lx) = (n-i)/(n+1)
  for (int i=0;i<n;++i)
    x[i] =  -1.0*vcl_log(double(n-i)/(n+1))/L;
}


//=======================================================================


//: Return a reference to the pdf model
// This is properly cast.
const pdf1d_exponential& pdf1d_exponential_sampler::exponential() const
{
  return (const pdf1d_exponential&) model();
}

