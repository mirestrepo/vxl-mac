// This is brl/bseg/boxm2/view/boxm2_ocl_render_tableau.h
#ifndef boxm2_ocl_render_tableau_h
#define boxm2_ocl_render_tableau_h
//:
// \file
// \brief A tableau to render using new processes.
// \author Vishal Jain
// \date Mar 25, 2011

#include "boxm2_include_glew.h"
#include "boxm2_cam_tableau.h"

//vgui includes
#include <vgui/vgui_gl.h>
#include <vgui/vgui_event_condition.h>
#include <vgui/vgui_statusbar.h>
#include <vgui/vgui_tableau_sptr.h>

//utilities
#include <vpgl/vpgl_perspective_camera.h>
#include <bocl/bocl_utils.h>
#include <vil/vil_convert.h>
#include <vil/vil_image_view_base.h>
#include <vil/vil_image_view.h>
#include <vil/vil_math.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vnl/vnl_random.h>

//boxm2 includes
#include <boxm2/boxm2_scene.h>
#include <boxm2/boxm2_data.h>
#include <boxm2/ocl/boxm2_opencl_cache.h>
#include <bocl/bocl_manager.h>
//brdb stuff
#include <brdb/brdb_value.h>

class boxm2_ocl_render_tableau : public boxm2_cam_tableau
{
 public:
  boxm2_ocl_render_tableau();
  virtual ~boxm2_ocl_render_tableau() {}
  
  //: initialize tableau with scene_file, viewport size, initial cam,  
  bool init(vcl_string scene_file,
            unsigned ni,
            unsigned nj,
            vpgl_perspective_camera<double>* cam);
  
  //: virtual function handles mouse and keyboard actions
  virtual bool handle( vgui_event const& e );

  //;  set the GL buffer which needs to be displayed.
  void set_glbuffer(GLuint  pbuffer) { pbuffer_ = pbuffer; }
  void set_statusbar(vgui_statusbar* status) { status_ = status; }

 protected:

  //vector of image files, vector of 

  bocl_manager_child_sptr mgr_;
  bocl_device_sptr device_;
  cl_command_queue queue_;
  //: Boxm2 Scene
  boxm2_scene_sptr scene_;
  boxm2_opencl_cache_sptr opencl_cache_;
  unsigned ni_;
  unsigned nj_;
  vgui_statusbar* status_;
  
  //: shared GL_CL image buffer
  GLuint pbuffer_;
  cl_mem clgl_buffer_;
  bocl_mem*  exp_img_;
  bocl_mem * exp_img_dim_;

  //--Render, update, refine, save helper methods ------------------------------
  //func to render frame on GPU (returns gpu time)
  float render_frame();

  bool init_clgl();
  bool do_init_ocl;
  
  int curr_frame_; 

};

//: declare smart pointer
typedef vgui_tableau_sptr_t<boxm2_ocl_render_tableau> boxm2_ocl_render_tableau_sptr;

//: Create a smart-pointer to a boxm2_ocl_render_tableau tableau.
struct boxm2_ocl_render_tableau_new : public boxm2_ocl_render_tableau_sptr
{
  //: Constructor - create an empty vgui_easy3D_tableau.
  typedef boxm2_ocl_render_tableau_sptr base;
  boxm2_ocl_render_tableau_new() : base( new boxm2_ocl_render_tableau ) { }
};

#endif // boxm2_ocl_render_tableau_h