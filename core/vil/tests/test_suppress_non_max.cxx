// This is core/vil2/tests/test_suppress_non_max.cxx
#include <testlib/testlib_test.h>
#include <vcl_iostream.h>
#include <vil2/algo/vil2_suppress_non_max.h>

void test_suppress_non_max_byte()
{
  vcl_cout << "***************************\n"
           << " Testing vil2_suppress_non_max\n"
           << "***************************\n";

  vil2_image_view<vxl_byte> image0,dest_im;
  image0.set_size(10,10);
  image0.fill(5);

  image0(3,7)=18;  // One peak
  image0(7,5)=19;  // Another peak

  image0(4,3)=20;  // Another peak at a plateau
  image0(4,4)=20;  // Another peak at a plateau

  vil2_suppress_non_max_3x3(image0,dest_im,vxl_byte(3));
  TEST("Peak at (3,7)",dest_im(3,7),18);
  TEST("Peak at (7,5)",dest_im(7,5),19);
  TEST("Flat Peak at (4,4) removed (sadly)",dest_im(4,4),0);

  TEST("No peak at 2,2",dest_im(2,2),0);
  TEST("No peak at 0,1",dest_im(3,3),0);
}

MAIN( test_suppress_non_max )
{
  START( "vil2_suppress_non_max_3x3" );

  test_suppress_non_max_byte();

  SUMMARY();
}
