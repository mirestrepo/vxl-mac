#ifndef vnl_matrix_exp_h_
#define vnl_matrix_exp_h_
#ifdef __GNUC__
#pragma interface
#endif
// This is vxl/vnl/vnl_matrix_exp.h

//: \file
// \brief Compute the exponential of a square matrix
//
// Compute the exponential of a square matrix, by summing its
// exponential series $exp(X) = \displaystyle\sum_{n \ge 0} X^n/n!$
// till a convergence requirement is met.
//
// Many improvements are possible.
//
//  \author fsm@robots.ox.ac.uk

#include <vnl/vnl_matrix.h>

//: Compute the exponential of a sqaure matrix - fiddly form
template <class T>
bool vnl_matrix_exp(vnl_matrix<T> const &X, vnl_matrix<T> &expX, double max_err);


//: Compute the exponential of a sqaure matrix - easy form.
template <class T>
vnl_matrix<T> vnl_matrix_exp(vnl_matrix<T> const &X);

#endif // vnl_matrix_exp_h_
