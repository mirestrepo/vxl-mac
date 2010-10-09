// This is tbl/vepl1/tests/vepl1_test_threshold.cxx

//:
// \file
//  Test of the vepl1_threshold templated IP classes
//  vepl1_threshold<vil1_image,vil1_image,T,T>
//  for T in { vxl_byte, vxl_uint_16, float }.
//
// \author Peter Vanroose, K.U.Leuven, ESAT/PSI
// \date   12 Sept. 2000

#include <vepl1/vepl1_threshold.h>
#include "test_driver.h"
#include <vxl_config.h> // for vxl_byte

#define ALL_TESTS(x,v,m) \
  ONE_TEST(x,byte_img,byte_ori,vxl_byte,v,m+"_byte"); \
  ONE_TEST(x,shrt_img,shrt_ori,vxl_uint_16,v,m+"_short")
#if 0 // Initially also had:
  ONE_TEST(x,flot_img,flot_ori,float,v,m+"_float")
  ONE_TEST(x,colr_img,colr_ori,vil1_rgb_byte,v,m+"_colour")
#endif // 0

int vepl1_test_threshold()
{
  vil1_image byte_img = CreateTest8bitImage(32,32),  byte_ori = CreateTest8bitImage(32,32);
  vil1_image shrt_img = CreateTest16bitImage(32,32), shrt_ori = CreateTest16bitImage(32,32);
#if 0 // Initially also had:
  vil1_image flot_img = CreateTestfloatImage(32,32), flot_ori = CreateTestfloatImage(32,32);
  vil1_image colr_img = CreateTest24bitImage(32,32), colr_ori = CreateTest24bitImage(32,32);
#endif // 0
  vcl_string m = "vepl1_threshold";
  ALL_TESTS(vepl1_threshold,0,m);
  return 0;
}

TESTMAIN(vepl1_test_threshold);
