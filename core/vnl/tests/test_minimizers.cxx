

#include <vcl_iostream.h>
#include <vcl_cassert.h>

#include <vnl/vnl_test.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matops.h>

#include <vnl/algo/vnl_brent.h>

struct cubic : public vnl_cost_function {
  cubic() : vnl_cost_function(1) {}

  double f(const vnl_vector<double>& x) {
    vcl_cout << " " << x[0];
    return (2 - x[0]) * (2 - x[0]) + 10;
  }
};

void test_minimizers()
{
  cubic c;
  vnl_brent b(&c);
  double x = 77;
  vcl_cout << "brent1: ";
  b.minimize_given_bounds(-100, x, 100, 1e-6, &x);
  vcl_cout << vcl_endl;
  TEST("brent1", vcl_fabs(x - 2) < 1e-5, true);
  vcl_cout << "brent2: ";
  x = 77;
  x = b.minimize(x);
  vcl_cout << vcl_endl;
  TEST("brent2", vcl_fabs(x - 2) < 1e-5, true);
}

TESTMAIN(test_minimizers);
