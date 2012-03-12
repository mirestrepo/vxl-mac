// This is brl/bseg/boxm2/pro/processes/boxm2_create_scene_process.cxx
#include <bprb/bprb_func_process.h>
//:
// \file
// \brief  A process for creating a scene
//
// \author Vishal Jain
// \date Mar 15, 2011

#include <vcl_fstream.h>
#include <vul/vul_file.h>
#include <boxm2/boxm2_scene.h>

namespace boxm2_create_scene_process_globals
{
  const unsigned n_inputs_ = 10;
  const unsigned n_outputs_ = 1;
}

bool boxm2_create_scene_process_cons(bprb_func_process& pro)
{
  using namespace boxm2_create_scene_process_globals;

  //process takes 9 inputs
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[0] = "vcl_string";
  input_types_[1] = "vcl_string";
  input_types_[2] = "vcl_string";
  input_types_[3] = "float"; // origin x
  input_types_[4] = "float"; // origin y
  input_types_[5] = "float"; // origin z
  input_types_[6] = "float"; // lon
  input_types_[7] = "float"; // lat
  input_types_[8] = "float"; // elev
  input_types_[9] = "int";   // number of illumination bins in the scene

  // process has 1 output
  vcl_vector<vcl_string>  output_types_(n_outputs_);
  output_types_[0] = "boxm2_scene_sptr";

  // ill bins might not be set
  brdb_value_sptr idx = new brdb_value_t<int>(0);
  brdb_value_sptr idx2 = new brdb_value_t<float>(0);
  pro.set_input(9, idx);
  pro.set_input(8, idx2);
  pro.set_input(7, idx2);
  pro.set_input(6, idx2);

  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}

bool boxm2_create_scene_process(bprb_func_process& pro)
{
  using namespace boxm2_create_scene_process_globals;

  if ( pro.n_inputs() < n_inputs_ ){
    vcl_cout << pro.name() << ": The input number should be " << n_inputs_<< vcl_endl;
    return false;
  }
  //get the inputs
  vcl_vector<vcl_string> appearance(2,"");
  unsigned i = 0;
  vcl_string datapath = pro.get_input<vcl_string>(i++);
  appearance[0]       = pro.get_input<vcl_string>(i++); //Appearance Model String
  appearance[1]       = pro.get_input<vcl_string>(i++); //Occupancy Model String
  float origin_x      = pro.get_input<float>(i++);
  float origin_y      = pro.get_input<float>(i++);
  float origin_z      = pro.get_input<float>(i++);
  float lon           = pro.get_input<float>(i++);
  float lat           = pro.get_input<float>(i++);
  float elev          = pro.get_input<float>(i++);
  int num_bins        = pro.get_input<int>(i++);

  if (!vul_file::make_directory_path(datapath.c_str()))
    return false;
  boxm2_scene_sptr scene =new boxm2_scene(datapath,vgl_point_3d<double>(origin_x,origin_y,origin_z));
  scene->set_local_origin(vgl_point_3d<double>(origin_x,origin_y,origin_z));
  scene->set_appearances(appearance);
  vpgl_lvcs lv = scene->lvcs();
  lv.set_origin((double)lon, (double)lat, (double)elev);
  scene->set_lvcs(lv);
  scene->set_num_illumination_bins(num_bins);
  i=0;  // store scene smart pointer
  pro.set_output_val<boxm2_scene_sptr>(i++, scene);
  return true;
}
