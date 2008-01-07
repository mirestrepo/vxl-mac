#include "vpro_fourier_process.h"
//:
// \file
#include <vcl_iostream.h>
#include <brip/brip_vil1_float_ops.h>
#include <vil1/vil1_memory_image_of.h>

bool vpro_fourier_process::execute()
{
  if (this->get_N_input_images()!=1)
  {
    vcl_cout << "In vpro_fourier_process::execute() -"
             << " not at exactly one input image\n";
    return false;
  }
  vil1_image img = vpro_video_process::get_input_image(0);
  this->clear_input();

  vil1_memory_image_of<float> fimg = brip_vil1_float_ops::convert_to_float(img);
  vil1_memory_image_of<float> pow_two_img;
  brip_vil1_float_ops::resize_to_power_of_two(fimg, pow_two_img);
  //compute fourier transform
  vil1_memory_image_of<float> mag, phase;
  brip_vil1_float_ops::fourier_transform(pow_two_img, mag, phase);

  //convert magnitude component to byte
  output_image_ = brip_vil1_float_ops::convert_to_byte(mag, -range_, range_);
  return true;
}
