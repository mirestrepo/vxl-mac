// This is tbl/vepl2/tests/vepl2_test_erode_disk.cxx

//:
// \file
//  Test of the vepl2_erode_disk function.
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   7 October 2002, from vepl/tests

#include <vepl2/vepl2_erode_disk.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include "test_driver.h"

int vepl2_test_erode_disk() {
  vcl_cout << "Starting vepl2_erode_disk tests\n";
  vcl_cout << "Creating test and output images ...";
  vil2_image_view_base_sptr byte_img = CreateTest8bitImage(32,32),  byte_ori = CreateTest8bitImage(32,32);
  vil2_image_view_base_sptr shrt_img = CreateTest16bitImage(32,32), shrt_ori = CreateTest16bitImage(32,32);
  vil2_image_view_base_sptr flot_img = CreateTestfloatImage(32,32), flot_ori = CreateTestfloatImage(32,32);
  vcl_cout << " done\n";

  vcl_string m = "vepl2_erode_disk";
#define args ,5
  ONE_TEST(vepl2_erode_disk,byte_img,byte_ori,unsigned char,233,m+"_byte",args);
  ONE_TEST(vepl2_erode_disk,shrt_img,shrt_ori,unsigned short,65257,m+"_short",args);
  ONE_TEST(vepl2_erode_disk,flot_img,flot_ori,float,338,m+"_float",args);

  return 0;
}

TESTMAIN(vepl2_test_erode_disk);
