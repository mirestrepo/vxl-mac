// This is vxl/vnl/vnl_cost_function.cxx

//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// vnl_cost_function
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 23 Oct 97
//
//-----------------------------------------------------------------------------

#include <vcl_cassert.h>
#include "vnl_cost_function.h"

static bool f_calling_compute;

void vnl_cost_function::compute(const vnl_vector<double>& x, double *f, vnl_vector<double>* g)
{
  if (f) *f = this->f(x);
  if (g) this->gradf(x, *g);
}

//: Default implementation of f is compute...
double vnl_cost_function::f(const vnl_vector<double>& x)
{
  // if we get back here from compute, neither vf was implemented.
  if (f_calling_compute)
    assert(!"vnl_cost_function: RECURSION");
  double f;
  f_calling_compute = true;
  this->compute(x, &f, 0);
  f_calling_compute = false;
  return f;
}

//: Default implementation of gradf is to call compute
void vnl_cost_function::gradf(const vnl_vector<double>&x, vnl_vector<double>&g)
{
  if (f_calling_compute)
    assert(!"vnl_cost_function: RECURSION");
  f_calling_compute = true;
  this->compute(x, 0, &g);
  f_calling_compute = false;
}

//: Compute fd gradient
void vnl_cost_function::fdgradf(const vnl_vector<double>& x,
                                vnl_vector<double>& gradient,
                                double stepsize )
{
  vnl_vector<double> tx = x;
  double h = stepsize;
  for(int i = 0; i < dim; ++i) {

    double tplus = x[i] + h;
    tx[i] = tplus;
    double fplus = this->f(tx);

    double tminus = x[i] - h;
    tx[i] = tminus;
    double fminus = this->f(tx);

    gradient[i] = (fplus - fminus) / (tplus - tminus);
    tx[i] = x[i];
  }
}

vnl_vector<double> vnl_cost_function::gradf(const vnl_vector<double>& x)
{
  vnl_vector<double> g(dim);
  this->gradf(x, g);
  return g;
}

vnl_vector<double> vnl_cost_function::fdgradf(const vnl_vector<double>& x)
{
  vnl_vector<double> g(dim);
  this->fdgradf(x, g);
  return g;
}
