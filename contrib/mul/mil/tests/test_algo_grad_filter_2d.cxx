#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vnl/vnl_test.h>
#include <mil/algo/mil_algo_grad_filter_2d.h>
#include <vil/vil_byte.h>
#include <vcl_cmath.h> // for fabs()

void test_algo_grad_filter_2d_byte_float()
{
  vcl_cout << "********************************************" << vcl_endl;
  vcl_cout << " Testing mil_algo_grad_filter_2d byte-float" << vcl_endl;
  vcl_cout << "********************************************" << vcl_endl;

  mil_image_2d_of<unsigned char> src;
  mil_image_2d_of<float> gx,gy;

  int n = 10;
  src.resize(n,n);
  src.fill(1);
  // Create square in middle
  for (int y=3;y<=7;++y)
    for (int x=3;x<=7;++x) src(x,y)=9;

//  src.print_all(vcl_cout);

  mil_algo_grad_filter_2d<unsigned char, float> filter;
  filter.filter_xy_3x3(gx,gy,src);
//  gx.print_all(vcl_cout);
//  gy.print_all(vcl_cout);

  TEST("Left edge of gx is zero",vcl_fabs(gx(0,5))<1e-6,true);
  TEST("Left edge of gy is zero",vcl_fabs(gy(0,5))<1e-6,true);
  TEST("Right edge of gx is zero",vcl_fabs(gx(n-1,5))<1e-6,true);
  TEST("Right edge of gy is zero",vcl_fabs(gy(n-1,5))<1e-6,true);
  TEST("Bottom edge of gx is zero",vcl_fabs(gx(5,0))<1e-6,true);
  TEST("Bottom edge of gy is zero",vcl_fabs(gy(5,0))<1e-6,true);
  TEST("Top edge of gx is zero",vcl_fabs(gx(5,n-1))<1e-6,true);
  TEST("Top edge of gy is zero",vcl_fabs(gy(5,n-1))<1e-6,true);

  TEST("Centre of gx image is zero",vcl_fabs(gx(5,5))<1e-6,true);
  TEST("Centre of gy image is zero",vcl_fabs(gy(5,5))<1e-6,true);

  TEST("Left of square in gx image",vcl_fabs(gx(2,5)-8)<1e-6,true);
  TEST("Right of square in gx image",vcl_fabs(gx(7,5)+8)<1e-6,true);
  TEST("Top of square in gx image",vcl_fabs(gx(5,7))<1e-6,true);
  TEST("Bottom of square in gx image",vcl_fabs(gx(5,2))<1e-6,true);

  TEST("Left of square in gy image",vcl_fabs(gy(2,5))<1e-6,true);
  TEST("Right of square in gy image",vcl_fabs(gy(7,5))<1e-6,true);
  TEST("Top of square in gy image",vcl_fabs(gy(5,7)+8)<1e-6,true);
  TEST("Bottom of square in gy image",vcl_fabs(gy(5,2)-8)<1e-6,true);

}

void test_algo_grad_filter_2d_float_float()
{
  vcl_cout << "********************************************" << vcl_endl;
  vcl_cout << " Testing mil_algo_grad_filter_2d float-float" << vcl_endl;
  vcl_cout << "********************************************" << vcl_endl;

  mil_image_2d_of<float> src;
  mil_image_2d_of<float> gx,gy;

  int n = 10;
  src.resize(n,n);
  src.fill(1);
  // Create square in middle
  for (int y=3;y<=7;++y)
    for (int x=3;x<=7;++x) src(x,y)=9;

//  src.print_all(vcl_cout);

  mil_algo_grad_filter_2d<float, float> filter;
  filter.filter_xy_3x3(gx,gy,src);
//  gx.print_all(vcl_cout);
//  gy.print_all(vcl_cout);

  TEST("Left edge of gx is zero",vcl_fabs(gx(0,5))<1e-6,true);
  TEST("Left edge of gy is zero",vcl_fabs(gy(0,5))<1e-6,true);
  TEST("Right edge of gx is zero",vcl_fabs(gx(n-1,5))<1e-6,true);
  TEST("Right edge of gy is zero",vcl_fabs(gy(n-1,5))<1e-6,true);
  TEST("Bottom edge of gx is zero",vcl_fabs(gx(5,0))<1e-6,true);
  TEST("Bottom edge of gy is zero",vcl_fabs(gy(5,0))<1e-6,true);
  TEST("Top edge of gx is zero",vcl_fabs(gx(5,n-1))<1e-6,true);
  TEST("Top edge of gy is zero",vcl_fabs(gy(5,n-1))<1e-6,true);

  TEST("Centre of gx image is zero",vcl_fabs(gx(5,5))<1e-6,true);
  TEST("Centre of gy image is zero",vcl_fabs(gy(5,5))<1e-6,true);

  TEST("Left of square in gx image",vcl_fabs(gx(2,5)-8)<1e-6,true);
  TEST("Right of square in gx image",vcl_fabs(gx(7,5)+8)<1e-6,true);
  TEST("Top of square in gx image",vcl_fabs(gx(5,7))<1e-6,true);
  TEST("Bottom of square in gx image",vcl_fabs(gx(5,2))<1e-6,true);

  TEST("Left of square in gy image",vcl_fabs(gy(2,5))<1e-6,true);
  TEST("Right of square in gy image",vcl_fabs(gy(7,5))<1e-6,true);
  TEST("Top of square in gy image",vcl_fabs(gy(5,7)+8)<1e-6,true);
  TEST("Bottom of square in gy image",vcl_fabs(gy(5,2)-8)<1e-6,true);

}

void test_algo_grad_filter_2d()
{
  test_algo_grad_filter_2d_byte_float();
  test_algo_grad_filter_2d_float_float();
}


TESTMAIN(test_algo_grad_filter_2d);
