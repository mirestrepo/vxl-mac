#include "bvxm_update_process.h"

#include <brdb/brdb_value.h>
#include <bprb/bprb_parameters.h>

#include <vil/vil_image_view_base.h>
#include <vpgl/vpgl_camera.h>

#include <bvxm/bvxm_voxel_world.h>
#include <bvxm/bvxm_image_metadata.h>
#include <bvxm/bvxm_mog_grey_processor.h>


bvxm_update_process::bvxm_update_process()
{
  //process takes 4inputs
  //input[0]: The observation image
  //input[1]: The camera of the observation
  //input[2]: The voxel world
  //input[3]: The apperance model type :this input must be either apm_mog_grey or apm_mog_rgb
  //          any other string will initialize the value for apm_mog_grey
  //input[4]: The bin index to be updatet
  input_data_.resize(5,brdb_value_sptr(0));
  input_types_.resize(5);
  input_types_[0] = "vil_image_view_base_sptr";
  input_types_[1] = "vpgl_camera_double_sptr";
  input_types_[2] = "bvxm_voxel_world_sptr";
  input_types_[3] = "vcl_string";
  input_types_[4] = "unsigned";


  //output has 1 output
  //output[0] : The updated probability map
  //output[1] : The mask of image pixels used in update
  output_data_.resize(2,brdb_value_sptr(0));
  output_types_.resize(2);
  output_types_[0]= "vil_image_view_base_sptr";
  output_types_[1]= "vil_image_view_base_sptr";


}


bool bvxm_update_process::execute()
{

  // Sanity check
  if(!this->verify_inputs())
    return false;

  //get the inputs
  brdb_value_t<vil_image_view_base_sptr>* input0 = 
    static_cast<brdb_value_t<vil_image_view_base_sptr>* >(input_data_[0].ptr());
  vil_image_view_base_sptr img = input0->value();

  brdb_value_t<vpgl_camera_double_sptr>* input1 = 
    static_cast<brdb_value_t<vpgl_camera_double_sptr>* >(input_data_[1].ptr());
  vpgl_camera_double_sptr camera = input1->value();

  brdb_value_t<bvxm_voxel_world_sptr>* input2 = 
    static_cast<brdb_value_t<bvxm_voxel_world_sptr>* >(input_data_[2].ptr());
  bvxm_voxel_world_sptr world = input2->value();

  brdb_value_t<vcl_string>* input3 = 
    static_cast<brdb_value_t<vcl_string>* >(input_data_[3].ptr());
  vcl_string voxel_type = input3->value();

  brdb_value_t<unsigned>* input4 = 
    static_cast<brdb_value_t<unsigned>* >(input_data_[4].ptr());
  unsigned bin_index = input4->value();

  //create metadata:
  bvxm_image_metadata observation(img,camera);

  //update
  vil_image_view<float> prob_map(img->ni(),img->nj(),1);
  vil_image_view<bool> mask(img->ni(),img->nj(),1);
  
  bool result; 

  if (voxel_type == "apm_mog_rgb")
     result = world->update<APM_MOG_RGB>(observation, prob_map, mask, bin_index);
  else
     result = world->update<APM_MOG_GREY>(observation, prob_map, mask, bin_index);

  if(!result){
    vcl_cerr << "error bvxm_update_process: failed to update observation" << vcl_endl;
    return false;
  }

  //store output
  brdb_value_sptr output0 = 
    new brdb_value_t<vil_image_view_base_sptr>(new vil_image_view<float>(prob_map));
  output_data_[0] = output0;

    brdb_value_sptr output1 = 
    new brdb_value_t<vil_image_view_base_sptr>(new vil_image_view<bool>(mask));
  output_data_[1] = output1;

  return true;
}



