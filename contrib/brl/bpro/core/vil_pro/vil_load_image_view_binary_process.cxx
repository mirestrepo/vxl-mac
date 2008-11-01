// This is brl/bpro/core/vil_pro/vil_load_image_view_binary_process.cxx
#include "vil_load_image_view_binary_process.h"
//:
// \file

#include <bprb/bprb_parameters.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vil/vil_image_view_base.h>
#include <core/vil_pro/vil_io_image_view_base.h>

//: Constructor
vil_load_image_view_binary_process::vil_load_image_view_binary_process()
{
  //input
  input_data_.resize(1,brdb_value_sptr(0));
  input_types_.resize(1);
  input_types_[0]="vcl_string";

  //output
  output_data_.resize(1,brdb_value_sptr(0));
  output_types_.resize(1);
  output_types_[0]= "vil_image_view_base_sptr";
}


//: Destructor
vil_load_image_view_binary_process::~vil_load_image_view_binary_process()
{
}


//: Execute the process
bool
vil_load_image_view_binary_process::execute()
{
  // Sanity check
    if (!this->verify_inputs())
    return false;

  //Retrieve filename from input
  brdb_value_t<vcl_string>* input0 =
    static_cast<brdb_value_t<vcl_string>* >(input_data_[0].ptr());

  vcl_string image_filename = input0->value();

  vsl_b_ifstream is(image_filename);
  vil_image_view_base_sptr loaded_image;
  vsl_b_read(is, loaded_image);
  is.close();

  if ( !loaded_image ) {
    vcl_cerr << "Failed to load image file" << image_filename << vcl_endl;
    return false;
  }

  brdb_value_sptr output0 = new brdb_value_t<vil_image_view_base_sptr>(loaded_image);
  output_data_[0] = output0;

  return true;
}

