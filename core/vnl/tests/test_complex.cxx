// This is vxl/vnl/tests/test_complex.cxx

//:
// \file

#include <vcl_iostream.h>
#include <vcl_cmath.h>
#include <vcl_complex.h>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_sample.h>

#include <testlib/testlib_test.h>

//: inverse cosine for complex numbers, implemented by Peter.Vanroose@esat.kuleuven.ac.be
vcl_complex<double> tc_acos(vcl_complex<double> x);
#if 0
{
  double a = vcl_real(x), b = vcl_imag(x);
  // special cases:
  if (b==0 && a > 1)
    return vcl_complex<double>(0.0, vcl_log(a+vcl_sqrt(a*a-1))); // == acosh(a)
  else if (b==0 && a >= -1)
    return vcl_acos(a);

  // the general case:
  // the result c + d*i satisfies a = cos(c)*cosh(d), b = -sin(c)*sinh(d)
  // hence $\frac{a^2}{\cos^2(c)} - \frac{b^2}{\sin^2(c)} = 1$.
  double t = 0.5*(1+a*a+b*b-vcl_sqrt((a*a-1)*(a*a-1)+b*b*(b*b+2*a*a+2)));
  // this $t = \cos^2(c)$ solves the latter biquadratic equation and lies in [0,1].
  double aa = a/vcl_sqrt(t), bb = b/vcl_sqrt(1-t);
  return vcl_complex<double>(vcl_acos(vcl_sqrt(t)), vcl_log(vcl_abs(aa-bb)));
}
#endif

// make a vector with random, complex entries :
static void fill_rand(vcl_complex<double> *b, vcl_complex<double> *e) {
  for (vcl_complex<double> *p=b; p<e; ++p)
    (*p) = vcl_complex<double>( vnl_sample_uniform(-1, +1), vnl_sample_uniform(-1, +1) );
}

// Driver
void test_complex() {
  {
    vcl_complex<double> a(-5), b(7,-1), c;
    c = a + b;
    c = a - b;
    c = a * b;
    c = a / b;
    a += b;
    a -= b;
    a *= b;
    a /= b;
    vcl_cout << "a=" << a << '\n'
             << "b=" << b << '\n'
             << "c=" << c << '\n'
             << '\n';
  }

  {
    vnl_vector<vcl_complex<double> > a(5); fill_rand(a.begin(), a.end());
    vnl_vector<vcl_complex<double> > b(5); fill_rand(b.begin(), b.end());

    vcl_cout << "a=" << a << '\n'
             << "b=" << b << '\n';

    vcl_complex<double> i(0,1);

    vcl_cout << dot_product(a,b) << '\n';
    testlib_test_assert_near("inner_product() conjugates correctly",
                             inner_product(i*a,b), i*inner_product(a,b));
    testlib_test_assert_near("inner_product() conjugates correctly",
                             inner_product(a,i*b),-i*inner_product(a,b));

    testlib_test_assert_near("dot_product() does not conjugate",
                             dot_product(i*a,b), i*dot_product(a,b));
    testlib_test_assert_near("dot_product() does not conjugate",
                             dot_product(a,i*b), i*dot_product(a,b));

    double norma=0;
    for (unsigned n=0; n<a.size(); ++n)
      norma += vcl_real(a[n])*vcl_real(a[n]) + vcl_imag(a[n])*vcl_imag(a[n]);
    norma = vcl_sqrt(norma);
    testlib_test_assert_near("correct magnitude", norma, a.magnitude());
  }

  int seed = 12345;
  for (int i=0; i<20; ++i)
  {
    seed = (seed*16807)%2147483647L;
    double u = double(seed)/1367130552L;
    if (u<0) u = -u; // between 0 and pi/2
    seed = (seed*16807)%2147483647L;
    double v = double(seed)/1000000000L;
    vcl_complex<double> c(u,v);
    vcl_complex<double> d = cos(c);
    vcl_complex<double> e = tc_acos(d);
    vcl_cout << c << ' ' << d << ' ' << e << '\n';
    testlib_test_assert_near("acos", c, e, 1e-12);
  }
}

TESTMAIN(test_complex);

vcl_complex<double> tc_acos(vcl_complex<double> x)
{
  double a = vcl_real(x), b = vcl_imag(x);
  // special cases:
  if (b==0 && a > 1)
    return vcl_complex<double>(0.0, vcl_log(a+vcl_sqrt(a*a-1))); // == acosh(a)
  else if (b==0 && a >= -1)
    return vcl_acos(a);

  // the general case:
  // the result c + d*i satisfies a = cos(c)*cosh(d), b = -sin(c)*sinh(d)
  // hence $\frac{a^2}{\cos^2(c)} - \frac{b^2}{\sin^2(c)} = 1$.
  double t = 0.5*(1+a*a+b*b-vcl_sqrt((a*a-1)*(a*a-1)+b*b*(b*b+2*a*a+2)));
  // this $t = \cos^2(c)$ solves the latter biquadratic equation and lies in [0,1].
  double aa = a/vcl_sqrt(t), bb = b/vcl_sqrt(1-t);
  double tmp = vcl_abs(aa-bb);
  return vcl_complex<double>(vcl_acos(vcl_sqrt(t)), vcl_log(tmp));
}
