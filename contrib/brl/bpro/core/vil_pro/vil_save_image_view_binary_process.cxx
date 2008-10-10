// This is brl/bpro/core/vil_pro/vil_save_image_view_binary_process.cxx
#include "vil_save_image_view_binary_process.h"
//:
// \file

#include <bprb/bprb_parameters.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vil/vil_save.h>
#include <vil/vil_image_view_base.h>
#include <vil/vil_math.h>
#include <vil/vil_convert.h>
#include <core/vil_pro/vil_io_image_view_base.h>

//: Constructor
vil_save_image_view_binary_process::vil_save_image_view_binary_process()
{
  //this process takes two inputs:
  // input(0): the vil_image_view_base_sptr
  // input(1): the filename to save to

  input_data_.resize(2,brdb_value_sptr(0));
  input_types_.resize(2);
  input_types_[0]="vil_image_view_base_sptr";
  input_types_[1]="vcl_string";

  //this process has no outputs
  output_data_.resize(0);
  output_types_.resize(0);
}
//: Destructor
vil_save_image_view_binary_process::~vil_save_image_view_binary_process()
{
}


//: Execute the process
bool
vil_save_image_view_binary_process::execute()
{
  // Sanity check
  if (!this->verify_inputs())
    return false;

  //Retrieve image from input
  brdb_value_t<vil_image_view_base_sptr>* input0 =
    static_cast<brdb_value_t<vil_image_view_base_sptr>* >(input_data_[0].ptr());

  vil_image_view_base_sptr img = input0->value();

  //Retrieve filename from input
  brdb_value_t<vcl_string>* input1 =
    static_cast<brdb_value_t<vcl_string>* >(input_data_[1].ptr());

  vcl_string image_filename = input1->value();

  vsl_b_ofstream os(image_filename);
  vsl_b_write(os, img);
  os.close();

  return true;
}

