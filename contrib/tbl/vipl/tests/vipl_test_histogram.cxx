//:
// \file
//  Test of the vipl_histogram templated IP classes
//  vipl_histogram<vil1_image,vnl_vector,T,unsigned>
//  for T in { unsigned char, unsigned short, float }.
//
// \author Ian Scott, ISBE, Manchester.
// \date   22 May 2002
//

#include <vil1/vil1_memory_image_of.h>
#include <vipl/accessors/vipl_accessors_vil1_image.h>
#include <vipl/accessors/vipl_accessors_vcl_vector.h>
#include <vipl/vipl_with_vnl_matrix/accessors/vipl_accessors_vnl_vector.h>
#include <vipl/vipl_histogram.h>
#include <vnl/vnl_vector.h>
#include <vcl_iostream.h>
#include "test_driver.h"


//create and image containing no 0s, one 1, two 2s, three 3s, etc.
vil1_image vipl_test_histogram_CreateTest8bitImage(int wd, int ht)
{
  unsigned char i =0, j=1;
  vil1_memory_image_of<unsigned char> image(wd, ht);
  for (int x = 0; x < wd; x++) {
    for (int y = 0; y < ht; y++) {
//    vcl_printf("%3d", j);
      image.put_section(&j, x, y, 1, 1);
      i++;
      if (i >= j) {j++;i=0;}
    }
//  vcl_printf("\n");
  }
  return image;
}


int vipl_test_histogram() {
  vil1_image byte_img = vipl_test_histogram_CreateTest8bitImage(32, 32);

  vcl_cout << "Starting vipl_histogram test\n";

  {
    const unsigned expected_n = 45;
    const int scale = 2;
    vnl_vector<unsigned> byte_out(expected_n / scale + 1);
    vipl_histogram<vil1_image,vnl_vector<unsigned>, unsigned char,unsigned> op(scale);
    op.put_in_data_ptr(&byte_img); op.put_out_data_ptr(&byte_out); op.filter();

    // check that final bin has correct number.
    bool test_pass = byte_out(byte_out.size()-1) + ((expected_n-2)*(expected_n-1))/2
                     == (unsigned int)(byte_img.width() * byte_img.height());

    // check that rest of bins are correct
    for (unsigned int i=0; i+2<byte_out.size(); ++i)
      if (byte_out(i)  != i*2 + i*2+1) test_pass = false;
    TEST("vnl_vector histogram counted correctly", test_pass, true);
    vcl_cout << byte_out << vcl_endl;
  }
  {
    vcl_vector<unsigned> byte_out(46);
    vipl_histogram<vil1_image,vcl_vector<unsigned>, unsigned char,unsigned> op;
    op.put_in_data_ptr(&byte_img); op.put_out_data_ptr(&byte_out); op.filter();

    // check that final bin has correct number.
    bool test_pass = byte_out.back() + ((byte_out.size()-2)*(byte_out.size()-1))/2
                     == (unsigned int)(byte_img.width() * byte_img.height());

    // check that rest of bins are correct
    for (unsigned int i=0; i+1<byte_out.size(); ++i)
      if (byte_out[i] != i) test_pass = false;
    TEST("vcl_vector histogram counted correctly", test_pass, true);
  }

  return 0;
}

TESTMAIN(vipl_test_histogram);
