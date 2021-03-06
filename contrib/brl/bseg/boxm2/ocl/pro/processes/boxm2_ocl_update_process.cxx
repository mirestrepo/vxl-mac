// This is brl/bseg/boxm2/ocl/pro/processes/boxm2_ocl_update_process.cxx
#include <bprb/bprb_func_process.h>
//:
// \file
// \brief  A process for updating the scene.
//
// \author Vishal Jain
// \date Mar 25, 2011

#include <vcl_fstream.h>
#include <vcl_algorithm.h>
#include <boxm2/ocl/boxm2_opencl_cache.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/boxm2_block.h>
#include <boxm2/boxm2_data_base.h>
#include <boxm2/ocl/boxm2_ocl_util.h>
#include <boxm2/boxm2_util.h>
#include <vil/vil_image_view.h>
#if 0
#include <vil/vil_save.h>
#endif
#include <vil/vil_new.h>
#include <vpl/vpl.h> // vpl_unlink()

#include <boxm2/ocl/algo/boxm2_ocl_camera_converter.h>
#include <boxm2/ocl/algo/boxm2_ocl_update.h>

//brdb stuff
#include <brdb/brdb_value.h>

//directory utility
#include <vcl_where_root_dir.h>
#include <bocl/bocl_device.h>
#include <bocl/bocl_kernel.h>

namespace boxm2_ocl_update_process_globals
{
  const unsigned int n_inputs_  = 9;
  const unsigned int n_outputs_ = 0;
}

bool boxm2_ocl_update_process_cons(bprb_func_process& pro)
{
  using namespace boxm2_ocl_update_process_globals;

  //process takes 9 inputs (of which the four last ones are optional):
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[0] = "bocl_device_sptr";
  input_types_[1] = "boxm2_scene_sptr";
  input_types_[2] = "boxm2_opencl_cache_sptr";
  input_types_[3] = "vpgl_camera_double_sptr";      //input camera
  input_types_[4] = "vil_image_view_base_sptr";     //input image
  input_types_[5] = "vcl_string";                   //illumination identifier
  input_types_[6] = "vil_image_view_base_sptr";     //mask image view
  input_types_[7] = "bool";                         //do_update_alpha/don't update alpha
  input_types_[8] = "float";                        //variance value? if 0.0 or less, then use variable variance

  // process has no outputs
  vcl_vector<vcl_string>  output_types_(n_outputs_);
  bool good = pro.set_input_types(input_types_) && pro.set_output_types(output_types_);

  // default 5, 6 and 7 and 8 inputs
  brdb_value_sptr idx        = new brdb_value_t<vcl_string>("");
  brdb_value_sptr empty_mask = new brdb_value_t<vil_image_view_base_sptr>(new vil_image_view<unsigned char>(1,1));
  brdb_value_sptr up_alpha   = new brdb_value_t<bool>(true);  //by default update alpha
  brdb_value_sptr def_var    = new brdb_value_t<float>(-1.0f);
  pro.set_input(5, idx);
  pro.set_input(6, empty_mask);
  pro.set_input(7, up_alpha);
  pro.set_input(8, def_var);
  return good;
}

bool boxm2_ocl_update_process(bprb_func_process& pro)
{
  using namespace boxm2_ocl_update_process_globals;
#if 0 // unused!
  vcl_size_t local_threads[2]={8,8};
  vcl_size_t global_threads[2]={8,8};
#endif
  //sanity check inputs
  if ( pro.n_inputs() < n_inputs_ ) {
    vcl_cout << pro.name() << ": The input number should be " << n_inputs_<< vcl_endl;
    return false;
  }
  //get the inputs
  unsigned int i = 0;
  bocl_device_sptr         device       = pro.get_input<bocl_device_sptr>(i++);
  boxm2_scene_sptr         scene        = pro.get_input<boxm2_scene_sptr>(i++);
  boxm2_opencl_cache_sptr  opencl_cache = pro.get_input<boxm2_opencl_cache_sptr>(i++);
  vpgl_camera_double_sptr  cam          = pro.get_input<vpgl_camera_double_sptr>(i++);
  vil_image_view_base_sptr img          = pro.get_input<vil_image_view_base_sptr>(i++);
  vcl_string               ident        = pro.get_input<vcl_string>(i++);
  vil_image_view_base_sptr mask_sptr    = pro.get_input<vil_image_view_base_sptr>(i++);
  bool                     update_alpha = pro.get_input<bool>(i++);
  float                    mog_var      = pro.get_input<float>(i++);

  //TODO Factor this out to a utility function
  //make sure this image small enough (or else carve it into image pieces)
  const vcl_size_t MAX_PIXELS = 16777216;
  if (img->ni()*img->nj() > MAX_PIXELS) {
    vcl_size_t sni = RoundUp(img->ni(), 16);
    vcl_size_t snj = RoundUp(img->nj(), 16);
    unsigned int numSegI = 1;
    unsigned int numSegJ = 1;
    while ( sni*snj*2 > MAX_PIXELS ) {
      sni /= 2;
      snj /= 2;
      ++numSegI;
      ++numSegJ;
    }
    sni = RoundUp(sni, 16);
    snj = RoundUp(snj, 16);
    vil_image_resource_sptr ir = vil_new_image_resource_of_view(*img);

    //run update for each image make sure to input i/j
    for (unsigned int i=0; i<=numSegI; ++i) {
      for (unsigned int j=0; j<=numSegJ; ++j) {
        //make sure the view doesn't extend past the original image
        vcl_size_t startI = (vcl_size_t) i * sni;
        vcl_size_t startJ = (vcl_size_t) j * snj;
        vcl_size_t endI = vcl_min(startI + sni, (vcl_size_t) img->ni());
        vcl_size_t endJ = vcl_min(startJ + snj, (vcl_size_t) img->nj());
        if (endI <= startI || endJ <= startJ)
          break;
        vcl_cout<<"Getting patch: ("<<startI<<','<<startJ<<") -> ("<<endI<<','<<endJ<<')'<<vcl_endl;
        vil_image_view_base_sptr view = ir->get_copy_view(startI, endI-startI, startJ, endJ-startJ);
#if 0
        //test saving
        vcl_stringstream s;
        s<<"block_"<<startI<<'_'<<startJ<<".png";
        vil_save(*view, s.str().c_str());
#endif
        //run update
        boxm2_ocl_update::update(scene, device, opencl_cache, cam, view,
                                 ident, mask_sptr, update_alpha, mog_var,
                                 startI, startJ);
      }
    }
    return true;
  }
  else //otherwise just run a normal update with one image
    return boxm2_ocl_update::update(scene, device, opencl_cache, cam, img, ident, mask_sptr, update_alpha, mog_var);
}
