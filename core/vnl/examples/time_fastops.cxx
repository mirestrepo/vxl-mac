#include <vcl_iostream.h>
#include <vcl_cmath.h> // for sqrt()
#include <vcl_vector.h>
#include <vbl/vbl_timer.h>

double vnl_fastops_dot(const double* a, const double* b, int n);

#ifdef OPTIMIZED
#undef OPTIMIZED
#define OPTIMIZED 1
#else
#define OPTIMIZED 0
#endif
#ifndef METHOD
#define METHOD 4
#endif

int main()
{
  vcl_vector<double> x(1000000), y(1000000);
  for(int i = 0; i < 1000000; ++i)
    x[i] = y[i] = 1.0/sqrt(double(i+1));
  
  vbl_timer t;
  for(int n = 0; n < 20; ++n)
    vnl_fastops_dot(&x[0], &y[0], x.size());
  vcl_cerr << "Method = " << METHOD << ", Optimized = " << OPTIMIZED << ", "
       << "Result = " << vnl_fastops_dot(&x[0], &y[0], x.size()) << ", ";
  t.print(vcl_cerr);
  
  return 0;
}

double vnl_fastops_dot(const double* a, const double* b, int n)
{
  double accum = 0;
#if METHOD == 1
  const double* aend = a + n;
  while (a != aend)
    accum += *a++ * *b++;
#endif
#if METHOD == 2
  for(int k = 0; k < n; ++k)
    accum += a[k] * b[k];
#endif
#if METHOD == 3
  while(n--)
    accum += a[n] * b[n];
#endif
#if METHOD == 4
  for(int k = n-1; k >= 0; --k)
    accum += a[k] * b[k];
#endif
  return accum;
}
