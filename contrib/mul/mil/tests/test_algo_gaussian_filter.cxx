#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_vector.h>

#include <vnl/vnl_test.h>
#include <mil/algo/mil_algo_gaussian_filter.h>
#include <vil/vil_byte.h>
#include <vcl_cstdlib.h> // for abs(int)
#include <vcl_cmath.h> // for fabs()
#include <vcl_algorithm.h>

void test_algo_gaussian_filter_byte_float()
{
  vcl_cout << "*********************************************" << vcl_endl;
  vcl_cout << " Testing mil_algo_gaussian_filter byte float" << vcl_endl;
  vcl_cout << "*********************************************" << vcl_endl;

  int n = 10;

  mil_image_2d_of<vil_byte> src(n,n);
  for (int i=0;i<n;++i)
    for (int j=0;j<n;++j)
      src(i,j)=j*10+i;
  mil_image_2d_of<float> dest;
  mil_algo_gaussian_filter<vil_byte,float> filter;
  filter.set_width(1.0);
  filter.filter(dest, src);
  vcl_cout << "Source\n";
  src.print_all(vcl_cout);
  vcl_cout << "Destination\n";
  dest.print_all(vcl_cout);


  bool fail=false;
  for (int i=2;i<n-2;++i)
    for (int j=2;j<n-2;++j)
      if (vcl_fabs(dest(i,j)-src(i,j)) > 0.5) fail = true;
  TEST("Central areas close to original", fail, false);

  fail=false;
  for (int i=0;i<n;++i)
    for (int j=0;j<n;++j)
      if (vcl_fabs(dest(i,j)-src(i,j)) > n/2)
        fail = true;
  TEST("Central areas not so close to original", fail, false);

  src.fill(0);
  src(n/2,n/2) = 255;

  filter.filter(dest, src);
  vcl_cout << "Source\n";
  src.print_all(vcl_cout);
  vcl_cout << "Destination\n";
  dest.print_all(vcl_cout);


  float sum = 0;
  bool fail1=false, fail2=false;
  for (int i=0;i<n;++i)
    for (int j=0;j<n;++j)
    {
      sum += dest(i,j);
      if (vcl_max(vcl_abs(i-n/2), vcl_abs(j-n/2)) > 2 && dest(i,j) != 0)
        fail1 = true;
      if (dest(i,j) > dest(n/2,n/2)) fail2=true;
    }
  TEST("impulse response is contained", fail1, false);
  TEST("response is centre maximum", fail2, false);
  TEST("impulse response sums to input", vcl_fabs(sum-255.0f) < 0.001, true);

}



void test_algo_gaussian_filter()
{
  test_algo_gaussian_filter_byte_float();
}


TESTMAIN(test_algo_gaussian_filter);
