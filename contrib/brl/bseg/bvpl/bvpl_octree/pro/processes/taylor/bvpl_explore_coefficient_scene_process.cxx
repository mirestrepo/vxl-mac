//:
// \brief A process to extract a taylor coefficient from the 10-d vector
// \file
// \author Isabel Restrepo
// \date 12-Apr-2011

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <bvpl/bvpl_octree/bvpl_global_taylor.h>

#include <vul/vul_file.h>

//:global variables
namespace bvpl_explore_coefficient_scene_process_globals 
{
  const unsigned n_inputs_ = 3;
  const unsigned n_outputs_ = 1;
}


//:sets input and output types
bool bvpl_explore_coefficient_scene_process_cons(bprb_func_process& pro)
{
  using namespace bvpl_explore_coefficient_scene_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i = 0;
  input_types_[i++] = "vcl_string"; //path to taylor info file
  input_types_[i++] = "int"; //scene id
  input_types_[i++] = "int"; //coefficient
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  output_types_[0] = "boxm_scene_base_sptr";
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool bvpl_explore_coefficient_scene_process(bprb_func_process& pro)
{
  using namespace bvpl_explore_coefficient_scene_process_globals;
  
  //get inputs
  vcl_string taylor_dir = pro.get_input<vcl_string>(0);
  int scene_id = pro.get_input<int>(1);
  int coeff_id = pro.get_input<int>(2);

  bvpl_global_taylor taylor(taylor_dir);
  
  boxm_scene_base_sptr valid_scene_base = taylor.load_valid_scene(scene_id);
  boxm_scene<boct_tree<short, bool> >* valid_scene = dynamic_cast<boxm_scene<boct_tree<short, bool> >*> (valid_scene_base.as_pointer());
  if (!valid_scene){
    vcl_cerr << "Error in bvpl_explore_coefficient_scene_process: Could not cast valid scene \n";
    return false;
  }
  
  vcl_stringstream scene_ss;
  scene_ss << "coefficient_" << coeff_id << "_scene_" << scene_id ;
  vcl_string scene_path = taylor.aux_dirs(scene_id) + "/" + scene_ss.str() + ".xml";
  
  vcl_cout << "Initializing " << scene_path << "\n";
  
  vcl_cout<< "Scene: " << scene_path << " does not exist, initializing xml " << vcl_endl;
  boxm_scene<boct_tree<short, float> > *scene =
  new boxm_scene<boct_tree<short, float> >(valid_scene->lvcs(), valid_scene->origin(), valid_scene->block_dim(), valid_scene->world_dim(), valid_scene->max_level(), valid_scene->init_level());
  scene->set_appearance_model(BOXM_FLOAT);
  scene->set_paths(taylor.aux_dirs(scene_id), scene_ss.str());
  scene->write_scene("/" + scene_ss.str() +  ".xml");
  taylor.extract_coefficient_scene(scene_id,  coeff_id, scene);

  
 
  //store output
  pro.set_output_val<boxm_scene_base_sptr>(0, scene);
  
  return true;
}