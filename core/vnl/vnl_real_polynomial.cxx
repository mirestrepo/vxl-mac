#ifdef __GNUC__
#pragma implementation
#endif
//
// Class: vnl_real_polynomial
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 23 Aug 96
//
//-----------------------------------------------------------------------------

#include "vnl_real_polynomial.h"
#include <vcl_complex.h>

// This is replacing a member template...
template <class T> 
T vnl_real_polynomial_evaluate(const double *a, int n, const T& x)
{
  --n;
  T acc = a[n];
  T xn = x;
  
  do {
    acc += a[--n] * xn;
    xn *= x;
  } while (n);
  
  return acc;  
}

#ifdef WIN32
#define SELECT(T) <T >
#else
#define SELECT(T) 
#endif

// Instantiate templates before use
template double         vnl_real_polynomial_evaluate SELECT(double        )(const double*, int, const double        &);
template vcl_complex<double> vnl_real_polynomial_evaluate SELECT(vcl_complex<double>)(const double*, int, const vcl_complex<double>&);

// -- Evaluate polynomial at value x
double vnl_real_polynomial::evaluate(double x) const
{
  return vnl_real_polynomial_evaluate SELECT(double)(coeffs_.data_block(), coeffs_.size(), x);
}

// -- Evaluate polynomial at complex value x
vcl_complex<double> vnl_real_polynomial::evaluate(const vcl_complex<double>& x) const
{
  return vnl_real_polynomial_evaluate SELECT(vcl_complex<double>)
     (coeffs_.data_block(), coeffs_.size(), x);
}

// -- Evaluate derivative at value x. Not implemented.
double vnl_real_polynomial::devaluate(double /*x*/) const
{
  return HUGE_VAL;
}

// -- Evaluate derivative at complex value x. Not implemented.
vcl_complex<double> vnl_real_polynomial::devaluate(const vcl_complex<double>& /*x*/) const
{
  return HUGE_VAL;
}
