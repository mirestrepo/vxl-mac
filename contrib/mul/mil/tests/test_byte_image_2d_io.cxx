#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_cmath.h>
#include <vil/vil_rgb_byte.h>

#include <vnl/vnl_test.h>
#include <mil/mil_convert_vil.h>
#include <vil/vil_memory_image_of.h>
#include <mil/mil_byte_image_2d_io.h>


void test_byte_image_2d_io()
{
  vcl_cout << "******************************" << vcl_endl;
  vcl_cout << " Testing mil_byte_image_2d_io" << vcl_endl;
  vcl_cout << "*******************************" << vcl_endl;

  int nx=20;
  int ny=20;
  mil_image_2d_of<vil_byte> orig_image(nx,ny),saved_image,loaded_image;
  // Fill image with shaded squares
  for (int y=0;y<ny;++y)
    for (int x=0;x<nx;++x)
      orig_image(x,y)=x*5+y*5;
  
  //orig_image.print_all(vcl_cout);


  //save the image
  saved_image=orig_image;
  mil_byte_image_2d_io image_io;
  image_io.saveTheImage(saved_image,"./tmp.bmp","bmp");
  image_io.saveTheImage(saved_image,"./tmp.jpg","jpeg");
  
  

  //load the image
  image_io.loadTheImage(loaded_image,"./tmp.bmp","bmp");
  image_io.saveTheImage(loaded_image,"./tmp_loaded.bmp","bmp");
  

  //nb have to use lossless image format ie bitmap, not JPEG!

  // Calc Total difference over all pixels
  double diff1=0;
  for (int y=0;y<ny;++y)
    for (int x=0;x<nx;++x)
      diff1+=vcl_fabs( saved_image(x,y)-loaded_image(x,y) );

#if 0
  vcl_cout<<"saved_image= "<<vcl_endl;
  saved_image.print_all(vcl_cout);

  vcl_cout<<"loaded_image= "<<vcl_endl;
  loaded_image.print_all(vcl_cout);

  vcl_cout<<"diff1= "<<diff1<<vcl_endl;
#endif

  TEST("loaded vs saved image",diff1<1e-6,true);
}

TESTMAIN(test_byte_image_2d_io);
