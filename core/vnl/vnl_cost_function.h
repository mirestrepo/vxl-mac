#ifndef vnl_cost_function_h_
#define vnl_cost_function_h_
#ifdef __GNUC__
#pragma interface
#endif
// This is vxl/vnl/vnl_cost_function.h

//:
//  \file
//  \brief Vector->Real function
//  \author Andrew W. Fitzgibbon, Oxford RRG
//  \date   23 Oct 97
//
// \verbatim
// Modifications
//  971023 AWF Initial version.
//  LSB (Manchester) 26/3/01 Tidied documentation
//   Feb.2002 - Peter Vanroose - brief doxygen comment placed on single line
// \endverbatim
//
//-----------------------------------------------------------------------------

#include <vnl/vnl_matops.h>
#include <vnl/vnl_unary_function.h>
#include <vnl/vnl_vector_ref.h>

//:   An object that represents a function from R^n -> R.
//    It is commonly used to express the
//    interface of a minimizer.
class vnl_cost_function : public vnl_unary_function<double, vnl_vector<double> > {
public:
  vnl_cost_function(int number_of_unknowns):dim(number_of_unknowns) {}

  virtual ~vnl_cost_function() {}

//:  The main function.  Given the parameter vector x, compute the value of f(x).
  virtual double f(vnl_vector<double> const& x);

//:  Calculate the gradient of f at parameter vector x.
  virtual void gradf(vnl_vector<double> const& x, vnl_vector<double>& gradient);

//:  Compute one or both of f and g.
//  Normally implemented in terms of the above two, but may be faster if specialized. f != 0 => compute f
  virtual void compute(vnl_vector<double> const& x, double *f, vnl_vector<double>* g);

//:  Return the number of unknowns
  int get_number_of_unknowns() const { return dim; }

//:  Compute finite-difference gradient
  void fdgradf(vnl_vector<double> const& x, vnl_vector<double>& gradient, double stepsize = 1e-5);

//:  Called when error is printed for user.
  virtual double reported_error(double f_value) { return f_value; }

//:  Conveniences for printing grad, fdgrad
  vnl_vector<double> gradf(vnl_vector<double> const& x);
  vnl_vector<double> fdgradf(vnl_vector<double> const& x);

public:
  int dim;
};


#endif // vnl_cost_function_h_
