#ifdef __GNUC__
#pragma implementation
#endif

// This is vxl/vnl/vnl_nonlinear_minimizer.cxx

//:
// \file
// \brief
// \author Andrew W. Fitzgibbon, Oxford RRG, 22 Aug 99
//

#include <vcl_cstdio.h>   // sprintf()
#include <vcl_iostream.h>
#include "vnl_nonlinear_minimizer.h"

//: Default ctor sets verbosity etc.
vnl_nonlinear_minimizer::vnl_nonlinear_minimizer()
{
  xtol = 1e-8;           // Termination tolerance on X (solution vector)
  maxfev = 2000; // Termination maximum number of iterations.
  ftol = xtol * 0.01;    // Termination tolerance on F (sum of squared residuals)
  gtol = 1e-5;           // Termination tolerance on Grad(F)' * F = 0
  epsfcn = xtol * 0.001; // Step length for FD Jacobian
  trace = false;
  verbose_ = false;

  reset();
}

vnl_nonlinear_minimizer::~vnl_nonlinear_minimizer()
{
}

vnl_matrix<double> const& vnl_nonlinear_minimizer::get_covariance()
{
  static vnl_matrix<double> null;
  return null;
}

void vnl_nonlinear_minimizer::reset()
{
  num_iterations_ = 0;
  num_evaluations_ = 0;
  start_error_ = 0;
}

//: Called by derived classes after each function evaluation.
void vnl_nonlinear_minimizer::report_eval(double f)
{
  if (num_evaluations_ == 0) {
    start_error_ = f;
    end_error_ = f;
  }
  if (f < end_error_)
    end_error_ = f;

  ++num_evaluations_;
}

//: Called by derived classes after each iteration
void vnl_nonlinear_minimizer::report_iter()
{
  ++num_iterations_;
  if (verbose_) {
    char buf[1024];
    vcl_sprintf(buf, "Iter %4d, Eval %4d: Best F = %10g\n",
                num_iterations_, num_evaluations_, end_error_);
    vcl_cerr << buf;
  }
}

//: Return the name of the class
//  Used by polymorphic IO
const vcl_string vnl_nonlinear_minimizer::is_a() const
{
  return vcl_string("vnl_nonlinear_minimizer");
}

