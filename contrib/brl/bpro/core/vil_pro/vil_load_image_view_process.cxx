// This is brl/bpro/core/vil_pro/vil_load_image_view_process.cxx
#include "vil_load_image_view_process.h"
//:
// \file

#include <bprb/bprb_parameters.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vil/vil_load.h>
#include <vil/vil_image_view_base.h>

//: Constructor
vil_load_image_view_process::vil_load_image_view_process()
{
  //input
  input_data_.resize(1,brdb_value_sptr(0));
  input_types_.resize(1);
  input_types_[0]="vcl_string";

  //output
  output_data_.resize(3,brdb_value_sptr(0));
  output_types_.resize(3);
  output_types_[0]= "vil_image_view_base_sptr";
  output_types_[1]= "unsigned";   // ni of the image
  output_types_[2]= "unsigned";   // nj of the image
}


//: Destructor
vil_load_image_view_process::~vil_load_image_view_process()
{
}


//: Execute the process
bool
vil_load_image_view_process::execute()
{
  // Sanity check
    if (!this->verify_inputs())
    return false;

  //Retrieve filename from input
  brdb_value_t<vcl_string>* input0 =
    static_cast<brdb_value_t<vcl_string>* >(input_data_[0].ptr());

  vcl_string image_filename = input0->value();

  vil_image_view_base_sptr loaded_image = vil_load(image_filename.c_str() );
  if ( !loaded_image ) {
    vcl_cerr << "Failed to load image file" << image_filename << vcl_endl;
    return false;
  }

  brdb_value_sptr output0 = new brdb_value_t<vil_image_view_base_sptr>(loaded_image);
  brdb_value_sptr output1 = new brdb_value_t<unsigned>(loaded_image->ni());
  brdb_value_sptr output2 = new brdb_value_t<unsigned>(loaded_image->nj());

  output_data_[0] = output0;
  output_data_[1] = output1;
  output_data_[2] = output2;
  return true;
}

