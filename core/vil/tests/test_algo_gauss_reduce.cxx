// This is mul/vil2/tests/test_algo_gauss_reduce.cxx
#include <testlib/testlib_test.h>
#include <vcl_iostream.h>
#include <vxl_config.h> // for vxl_byte
#include <vil2/vil2_print.h>
#include <vil2/vil2_image_view.h>
#include <vil2/algo/vil2_gauss_reduce.h>

void test_algo_gauss_reduce_byte(unsigned nx)
{
  vcl_cout << "*********************************************\n"
           << " Testing vil2_algo_gauss_reduce (byte)(nx="<<nx<<")\n"
           << "*********************************************\n";

  vil2_image_view<vxl_byte> image0;
  image0.resize(nx,3);
  vil2_image_view<vxl_byte> reduced_x;
  reduced_x.resize((nx+1)/2,3);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = i+j*10;

  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST("First element",reduced_x(0,1),image0(0,1));
  TEST("Next element", reduced_x(1,1),image0(2,1));
  unsigned L = (nx-1)/2;
  TEST("Last element", reduced_x(L,1),image0(2*L,1));

  vil2_image_view<vxl_byte> test2;
  test2.resize(nx,3);
  test2.fill(222);
  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("No overrun",test2(L+1,1),222);

  // Test it can be used to smooth in y by swapping ordinates
  vil2_image_view<vxl_byte> image1;
  image1.resize(3,nx);
  vil2_image_view<vxl_byte> reduced_y;
  reduced_y.resize(3,(nx+1)/2);

  for (unsigned int j=0;j<image1.nj();++j)
    for (unsigned int i=0;i<image1.ni();++i)
      image1(i,j) = i+j*10;

  vil2_gauss_reduce(image1.top_left_ptr(),image1.nj(),image1.ni(),
                     image1.jstep(),image1.istep(),
                     reduced_y.top_left_ptr(),reduced_y.jstep(),reduced_y.istep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image1); vcl_cout<<'\n';
  vcl_cout<<"reduced_y : "; vil2_print_all(vcl_cout,reduced_y); vcl_cout<<'\n';

  TEST("First element",reduced_y(1,0),image1(1,0));
  TEST("Next element", reduced_y(1,1),image1(1,2));
  TEST("Last element", reduced_y(1,L),image1(1,2*L));
}

void test_algo_gauss_reduce_int_32(unsigned nx)
{
  vcl_cout << "*********************************************\n"
           << " Testing vil2_algo_gauss_reduce (int_32)(nx="<<nx<<")\n"
           << "*********************************************\n";

  vil2_image_view<vxl_int_32> image0;
  image0.resize(nx,3);
  vil2_image_view<vxl_int_32> reduced_x;
  reduced_x.resize((nx+1)/2,3);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = i+j*10;

  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST("First element",reduced_x(0,1),image0(0,1));
  TEST("Next element", reduced_x(1,1),image0(2,1));
  unsigned L = (nx-1)/2;
  TEST("Last element", reduced_x(L,1),image0(2*L,1));

  vil2_image_view<vxl_int_32> test2;
  test2.resize(nx,3);
  test2.fill(222);
  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("No overrun",test2(L+1,1),222);

  // Test it can be used to smooth in y by swapping ordinates
  vil2_image_view<vxl_int_32> image1;
  image1.resize(3,nx);
  vil2_image_view<vxl_int_32> reduced_y;
  reduced_y.resize(3,(nx+1)/2);

  for (unsigned int j=0;j<image1.nj();++j)
    for (unsigned int i=0;i<image1.ni();++i)
      image1(i,j) = i+j*10;

  vil2_gauss_reduce(image1.top_left_ptr(),image1.nj(),image1.ni(),
                     image1.jstep(),image1.istep(),
                     reduced_y.top_left_ptr(),reduced_y.jstep(),reduced_y.istep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image1); vcl_cout<<'\n';
  vcl_cout<<"reduced_y : "; vil2_print_all(vcl_cout,reduced_y); vcl_cout<<'\n';

  TEST("First element",reduced_y(1,0),image1(1,0));
  TEST("Next element", reduced_y(1,1),image1(1,2));
  TEST("Last element", reduced_y(1,L),image1(1,2*L));
}

void test_algo_gauss_reduce_float(int nx)
{
  vcl_cout << "**********************************************\n"
           << " Testing vil2_algo_gauss_reduce (float)(nx="<<nx<<")\n"
           << "**********************************************\n";

  vil2_image_view<float> image0;
  image0.resize(nx,3);
  vil2_image_view<float> reduced_x;
  reduced_x.resize((nx+1)/2,3);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = 0.1f*i+j;

  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST_NEAR("First element",reduced_x(0,1),image0(0,1),1e-6);
  TEST_NEAR("Next element",reduced_x(1,1),image0(2,1),1e-6);
  unsigned L = (nx-1)/2;
  TEST_NEAR("Last element",reduced_x(L,1),image0(2*L,1),1e-6);


  vil2_image_view<float> test2;
  test2.resize(nx,3);
  test2.fill(22.2f);
  vil2_gauss_reduce(image0.top_left_ptr(),image0.ni(),image0.nj(),
                     image0.istep(),image0.jstep(),
                     test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST_NEAR("No overrun",test2(L+1,1),22.2f,1e-6);
}

void test_algo_gauss_reduce_121_byte(unsigned nx, unsigned ny)
{
  vcl_cout << "*******************************************************\n"
           << " Testing vil2_algo_gauss_reduce_121 (byte)(nx="<<nx<<", ny="<<ny<<")\n"
           << "*******************************************************\n";

  vil2_image_view<vxl_byte> image0;
  image0.resize(nx,ny);
  vil2_image_view<vxl_byte> reduced_x;
  reduced_x.resize((nx+1)/2,(ny+1)/2);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = i+j*10;

  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                         image0.istep(),image0.jstep(),
                         reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST("First element",reduced_x(0,1),image0(0,2));
  TEST("Next element",reduced_x(1,1),image0(2,2));
  unsigned Lx = (nx+1)/2;
  unsigned Ly = (ny+1)/2;
  TEST("Last element in x",reduced_x(Lx-1,1),image0(2*(Lx-1),2));
  TEST("Last element in y",reduced_x(1,Ly-1),image0(2,2*(Ly-1)));

  vil2_image_view<vxl_byte> test2;
  test2.resize(nx,ny);
  test2.fill(222);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                         image0.istep(),image0.jstep(),
                         test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("No overrun in x",test2(Lx,1), 222);
  TEST("No overrun in y",test2(1,Ly), 222);

  image0.fill(17);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                        image0.istep(),image0.jstep(),
                        test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("Smoothing correct",test2(1,1), 17);
  vcl_cout<<"Value at (1,1):"<<int(test2(1,1))<<'\n';
}


void test_algo_gauss_reduce_121_int_32(unsigned nx, unsigned ny)
{
  vcl_cout << "*******************************************************\n"
           << " Testing vil2_algo_gauss_reduce_121 (int32)(nx="<<nx<<", ny="<<ny<<")\n"
           << "*******************************************************\n";

  vil2_image_view<vxl_int_32> image0;
  image0.resize(nx,ny);
  vil2_image_view<vxl_int_32> reduced_x;
  reduced_x.resize((nx+1)/2,(ny+1)/2);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = i+j*10;

  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                         image0.istep(),image0.jstep(),
                         reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST("First element",reduced_x(0,1),image0(0,2));
  TEST("Next element",reduced_x(1,1),image0(2,2));
  unsigned Lx = (nx+1)/2;
  unsigned Ly = (ny+1)/2;
  TEST("Last element in x",reduced_x(Lx-1,1),image0(2*(Lx-1),2));
  TEST("Last element in y",reduced_x(1,Ly-1),image0(2,2*(Ly-1)));

  vil2_image_view<vxl_int_32> test2;
  test2.resize(nx,ny);
  test2.fill(222);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                         image0.istep(),image0.jstep(),
                         test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("No overrun in x",test2(Lx,1), 222);
  TEST("No overrun in y",test2(1,Ly), 222);

  image0.fill(17);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                        image0.istep(),image0.jstep(),
                        test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST("Smoothing correct",test2(1,1), 17);
  vcl_cout<<"Value at (1,1):"<<int(test2(1,1))<<'\n';
}


void test_algo_gauss_reduce_121_float(unsigned nx, unsigned ny)
{
  vcl_cout << "********************************************************\n"
           << " Testing vil2_algo_gauss_reduce_121 (float)(nx="<<nx<<", ny="<<ny<<")\n"
           << "********************************************************\n";

  vil2_image_view<float> image0;
  image0.resize(nx,ny);
  vil2_image_view<float> reduced_x;
  reduced_x.resize((nx+1)/2,(ny+1)/2);

  for (unsigned int j=0;j<image0.nj();++j)
    for (unsigned int i=0;i<image0.ni();++i)
      image0(i,j) = 0.1f*i+j;

  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                             image0.istep(),image0.jstep(),
                             reduced_x.top_left_ptr(),reduced_x.istep(),reduced_x.jstep());

  vcl_cout<<"Original: "; vil2_print_all(vcl_cout,image0); vcl_cout<<'\n';
  vcl_cout<<"reduced_x : "; vil2_print_all(vcl_cout,reduced_x); vcl_cout<<'\n';

  TEST_NEAR("First element",reduced_x(0,1),image0(0,2),1e-6);
  TEST_NEAR("Next element",reduced_x(1,1),image0(2,2),1e-6);
  unsigned Lx = (nx+1)/2;
  unsigned Ly = (ny+1)/2;
  TEST_NEAR("Last element in x",reduced_x(Lx-1,1),image0(2*(Lx-1),2),1e-6);
  TEST_NEAR("Last element in y",reduced_x(1,Ly-1),image0(2,2*(Ly-1)),1e-6);

  vil2_image_view<float> test2;
  test2.resize(nx,ny);
  test2.fill(22.2f);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                             image0.istep(),image0.jstep(),
                             test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST_NEAR("No overrun in x",test2(Lx,1),22.2f,1e-6);
  TEST_NEAR("No overrun in y",test2(1,Ly),22.2f,1e-6);

  image0.fill(1.7f);
  vil2_gauss_reduce_121(image0.top_left_ptr(),image0.ni(),image0.nj(),
                             image0.istep(),image0.jstep(),
                             test2.top_left_ptr(),test2.istep(),test2.jstep());
  TEST_NEAR("Smoothing correct",test2(1,1),1.7f,1e-6);
  vcl_cout<<"Value at (1,1):"<<float(test2(1,1))<<'\n';
}

void test_algo_gauss_reduce_byte_2d()
{
  vcl_cout<<"Testing reduction in 2D\n";
  unsigned ni = 20, nj = 20;

  vil2_image_view<vxl_byte> image0(ni,nj),image1,work_im;

  for (unsigned y=0;y<image0.nj();++y)
    for (unsigned x=0;x<image0.ni();++x)
      image0(x,y) = x+y*10;

  vil2_gauss_reduce(image0,image1,work_im);
  unsigned ni2 = (ni+1)/2;
  unsigned nj2 = (nj+1)/2;
  TEST("Level 1 size x",image1.ni(),(ni+1)/2);
  TEST("Level 1 size y",image1.nj(),(nj+1)/2);
  TEST("Pixel (0,0)",image0(0,0),image1(0,0));
  TEST("Pixel (1,1)",image0(2,2),image1(1,1));
  TEST("Corner pixel",image0(ni2*2-2,nj2*2-2),image1(ni2-1,nj2-1));
}

MAIN( test_algo_gauss_reduce )
{
  START( "vil2_algo_gauss_reduce" );

  test_algo_gauss_reduce_byte(7);
  test_algo_gauss_reduce_byte(6);
  test_algo_gauss_reduce_int_32(7);
  test_algo_gauss_reduce_int_32(6);
  test_algo_gauss_reduce_float(7);
  test_algo_gauss_reduce_float(6);

  test_algo_gauss_reduce_121_byte(6,6);
  test_algo_gauss_reduce_121_byte(7,7);
  test_algo_gauss_reduce_121_int_32(6,6);
  test_algo_gauss_reduce_121_int_32(7,7);
  test_algo_gauss_reduce_121_float(6,6);
  test_algo_gauss_reduce_121_float(7,7);

  test_algo_gauss_reduce_byte_2d();

  SUMMARY();
}
