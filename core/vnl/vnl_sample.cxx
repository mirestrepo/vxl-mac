/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation "vnl_sample"
#endif
#include "vnl_sample.h"

#include <vcl/vcl_cmath.h>
#include <vcl/vcl_cstdlib.h>

// -- return a random number uniformly drawn on [a, b)
double vnl_sample_uniform(double a, double b) 
{
#ifdef VCL_NO_DRAND48
    // rand() is not always a good random number generator,
    // so use the following congruential random number generator - PVr
  static unsigned long seed = 12345;
  seed = (seed*16807)%2147483647L;
  double u = seed/2147483711;
#else
    // If you don't have drand48() you must define 
    // the macro VCL_NO_DRAND48 somewhere.
  double u = drand48(); // uniform on [0, 1)
#endif
  return (1-u)*a + u*b;
}

double vnl_sample_normal(double mean, double sigma) 
{
  double u     = vnl_sample_uniform(0, 1);
  double theta = vnl_sample_uniform(0, 2*3.1415926);
  
  double r = sqrt(-2*log(u));
  
  double x = r * cos(theta);
#ifdef fred
  double y = r * sin(theta);
#endif
  
  return mean + sigma * x;
}
