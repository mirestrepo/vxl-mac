#include "boxm2_render_rgb_tableau.h"
//:
// \file
#include <boxm2/ocl/boxm2_ocl_util.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vpgl/vpgl_calibration_matrix.h>
#include <vgui/internals/trackball.h>
#include <vgui/vgui_modifier.h>
#include <vgl/vgl_distance.h>
#include <vcl_sstream.h>


//: Constructor
boxm2_render_rgb_tableau::boxm2_render_rgb_tableau()
{
  curr_frame_ = 0;
  pbuffer_=0;
  ni_=640;
  nj_=480;
  do_update_ = false;
}

//: initialize tableau properties
bool boxm2_render_rgb_tableau::init(vcl_string scene_file,
                                    unsigned ni,
                                    unsigned nj,
                                    vpgl_perspective_camera<double> * cam,
                                    vcl_vector<vcl_string> cam_files,
                                    vcl_vector<vcl_string> img_files)
{
  //set image dimensions, camera and scene
  ni_ = ni;
  nj_ = nj;
  cam_   = (*cam);
  default_cam_ = (*cam);
  do_init_ocl = true;
  update_count_ = 0;
  cam_files_ = cam_files;
  img_files_ = img_files;

  //: default image type is uchar;
  img_type_ ="uchar";

  //initialize transfer function...
  tf_sptr=new bbas_1d_array_float(256);
  mini_=0.0f;
  maxi_=1.0f;
  for (int i=0; i<256; ++i) tf_sptr->data_array[i] =(float)i/255.0f;
  
  //create the scene
  scene_ = new boxm2_scene(scene_file);
  vcl_cout<<*scene_<<vcl_endl;

  return true;
}


//: Handles tableau events (drawing and keys)
bool boxm2_render_rgb_tableau::handle(vgui_event const &e)
{
  //draw handler - called on post_draw()
  if (e.type == vgui_DRAW)
  {
    if (do_init_ocl) {
      this->init_clgl();
      do_init_ocl = false;
    }

    {
    float gpu_time = this->render_frame();
    this->setup_gl_matrices();
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glRasterPos2i(0, 1);
    glPixelZoom(1,-1);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, pbuffer_);
    glDrawPixels(ni_, nj_, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

    //calculate and write fps to status
    vcl_stringstream str;
    str<<"num updates: "<<update_count_
       <<".  rendering at ~ "<< (1000.0f / gpu_time) <<" fps ";
    status_->write(str.str().c_str());
    }
    return true;
  }

  //handle update command - keyboard press U
  else if (e.type == vgui_KEY_PRESS && e.key == vgui_key('u')) {
    vcl_cout<<"Continuing update"<<vcl_endl;
    do_update_ = true;
    this->post_idle_request();
  }

  else if (e.type == vgui_KEY_PRESS && e.key == vgui_key('s')) {
    vcl_cout<<"saving"<<vcl_endl;
    this->save_model();
    return true;
  }
  else if (e.type == vgui_KEY_PRESS && e.key == vgui_key('d')) {
    vcl_cout<<"refining"<<vcl_endl;
    this->refine_model();
    return true;
  }
  //HANDLE idle events - do model updating
  else if (e.type == vgui_IDLE)
  {
    if (do_update_) {
      this->update_frame();
      this->post_redraw();
      return true;
    }
    else {
      return false;
    }
  }
  //if you click on the canvas, you wanna render, so turn off update
  else if (e.type == vgui_BUTTON_DOWN) {
    do_update_ = false;
  }


  if (boxm2_cam_tableau::handle(e))
    return true;

  return false;
}

bool boxm2_render_rgb_tableau::save_model()
{
  vcl_cout<<"SAVING MODEL!!!"<<vcl_endl;

  //save blocks and data to disk for debugging
  vcl_vector<vcl_string> apps = scene_->appearances(); 
  vcl_map<boxm2_block_id, boxm2_block_metadata> blocks = scene_->blocks();
  vcl_map<boxm2_block_id, boxm2_block_metadata>::iterator iter;
  for (iter = blocks.begin(); iter != blocks.end(); ++iter)
  {
    boxm2_block_id id = iter->first;
    boxm2_sio_mgr::save_block(scene_->data_path(), cache_->get_block(id));
    boxm2_sio_mgr::save_block_data(scene_->data_path(), id, cache_->get_data<BOXM2_ALPHA>(id) );
    
    //save scene's appearance models (app and num obs)
    for(int i=0; i<apps.size(); ++i)
      boxm2_sio_mgr::save_block_data_base(scene_->data_path(), id, cache_->get_data_base(id, apps[i]), apps[i]);
  }
  return true;
}

//: refines model
float boxm2_render_rgb_tableau::refine_model()
{
    //create generic scene
    brdb_value_sptr brdb_scene = new brdb_value_t<boxm2_scene_sptr>(scene_);
    brdb_value_sptr brdb_data_type;
    if(img_type_=="uchar")
        brdb_data_type= new brdb_value_t<vcl_string>(vcl_string("8bit"));
    else if (img_type_=="ushort")
        brdb_data_type= new brdb_value_t<vcl_string>(vcl_string("16bit"));

    //set inputs
    vcl_vector<brdb_value_sptr> input;
    input.push_back(brdb_scene);
    input.push_back(brdb_data_type);

    //initoutput vector
    vcl_vector<brdb_value_sptr> output;

    //execute gpu_update
    gpu_pro_->run(&refine_, input, output);
  return gpu_pro_->exec_time();
}

//: calls on ray manager to render frame into the pbuffer_
float boxm2_render_rgb_tableau::render_frame()
{
  cl_int status = clEnqueueAcquireGLObjects( *gpu_pro_->get_queue(), 1,
                                             &render_.gl_image()->buffer() , 0, 0, 0);
  exp_img_->zero_gpu_buffer( *gpu_pro_->get_queue() );
  if (!check_val(status,CL_SUCCESS,"clEnqueueAcquireGLObjects failed. (gl_image)"+error_to_string(status)))
    return false;

  //set inputs
  vcl_cout<<cam_<<vcl_endl;
  vpgl_camera_double_sptr cam = new vpgl_perspective_camera<double>(cam_);
  brdb_value_sptr brdb_cam = new brdb_value_t<vpgl_camera_double_sptr>(cam);

  //create output image buffer (rgba float image)
  vil_image_view_base_sptr expimg = new vil_image_view<vil_rgba<float> >(ni_, nj_);
  brdb_value_sptr brdb_expimg = new brdb_value_t<vil_image_view_base_sptr>(expimg);

  //create vis image buffer
  vil_image_view<float>* visimg = new vil_image_view<float>(ni_, nj_);
  visimg->fill(1.0f);
  brdb_value_sptr brdb_visimg = new brdb_value_t<vil_image_view_base_sptr>(visimg);

  //create scene brdbvalue pointer
  brdb_value_sptr brdb_scene = new brdb_value_t<boxm2_scene_sptr>(scene_);
  brdb_value_sptr brdb_data_type = new brdb_value_t<vcl_string>(vcl_string("8bit"));
  brdb_value_sptr brdb_mini = new brdb_value_t<float>(mini_);
  brdb_value_sptr brdb_maxi = new brdb_value_t<float>(maxi_);
  brdb_value_sptr brdb_float_array = new brdb_value_t<bbas_1d_array_float_sptr>(tf_sptr);

  vcl_vector<brdb_value_sptr> input;
  input.push_back(brdb_scene);
  input.push_back(brdb_cam);
  input.push_back(brdb_expimg);
  input.push_back(brdb_visimg);
  input.push_back(brdb_data_type);
  input.push_back(brdb_mini);
  input.push_back(brdb_maxi);
  input.push_back(brdb_float_array);
  
  //initoutput vector
  vcl_vector<brdb_value_sptr> output;

  //initialize the GPU render process
  gpu_pro_->run(&render_, input, output);
  //gpu_pro_->finish();

  status = clEnqueueReleaseGLObjects( *gpu_pro_->get_queue(), 1, &render_.gl_image()->buffer(), 0, 0, 0);
  clFinish( *render_.command_queue() );
  return gpu_pro_->exec_time();
}

//: updates given a random frame
float boxm2_render_rgb_tableau::update_frame()
{
    update_count_++;
    if (update_count_%5==0)
    {
        //vcl_cout<<"Refining  "<<vcl_endl;
        //float time=this->refine_model();
    }
    //pickup a random frame
    int curr_frame = rand.lrand32(0,cam_files_.size()-1);
    //curr_frame_ = (curr_frame_+1 >= cam_files_.size()) ? 0 : curr_frame_+1;
    //int curr_frame = curr_frame_;
    vcl_cout<<"Cam "<<cam_files_[curr_frame]<<'\n'
            <<"Image "<<img_files_[curr_frame]<<vcl_endl;

    //build the camera from file
    vcl_ifstream ifs(cam_files_[curr_frame].c_str());
    vpgl_perspective_camera<double>* pcam = new vpgl_perspective_camera<double>;
    if (!ifs.is_open()) {
        vcl_cerr << "Failed to open file " << cam_files_[curr_frame] << '\n';
        return -1;
    }
    ifs >> *pcam;
    vpgl_camera_double_sptr cam_sptr(pcam);
    brdb_value_sptr brdb_cam = new brdb_value_t<vpgl_camera_double_sptr>(cam_sptr);

    //load image from file
    vil_image_view_base_sptr floatimg_sptr =  boxm2_ocl_util::prepare_input_image(img_files_[curr_frame]); 
    brdb_value_sptr brdb_inimg = new brdb_value_t<vil_image_view_base_sptr>(floatimg_sptr);

    //create generic scene
    brdb_value_sptr brdb_scene = new brdb_value_t<boxm2_scene_sptr>(scene_);
    brdb_value_sptr brdb_data_type = new brdb_value_t<vcl_string>(vcl_string("8bit"));

    //set inputs
    vcl_vector<brdb_value_sptr> input;
    input.push_back(brdb_scene);
    input.push_back(brdb_cam);
    input.push_back(brdb_inimg);
    input.push_back(brdb_data_type);

    //initoutput vector
    vcl_vector<brdb_value_sptr> output;

    //execute gpu_update
    gpu_pro_->run(&update_, input, output);
    return gpu_pro_->exec_time();
}

//: private helper method to init_clgl stuff (gpu processor)
bool boxm2_render_rgb_tableau::init_clgl()
{
  //get relevant blocks
  vcl_cout<<"Data Path: "<<scene_->data_path()<<vcl_endl;
  
  //create cache, grab singleton instance
  boxm2_lru_cache::create(scene_.ptr());
  cache_ = boxm2_cache::instance();

  //initialize gpu pro / manager
  gpu_pro_ = boxm2_opencl_processor::instance();
  gpu_pro_->context_ = create_clgl_context();
  gpu_pro_->set_scene(scene_.ptr());
  gpu_pro_->set_cpu_cache(cache_);
  gpu_pro_->init();

  // delete old buffer
  if (pbuffer_) {
      clReleaseMemObject(clgl_buffer_);
      glDeleteBuffers(1, &pbuffer_);
  }

  //generate glBuffer, and bind to ray_mgr->image_gl_buf_
  glGenBuffers(1, &pbuffer_);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbuffer_);
  glBufferData(GL_PIXEL_UNPACK_BUFFER, ni_*nj_*sizeof(GLubyte)*4, 0, GL_STREAM_DRAW);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

  //create OpenCL buffer from GL PBO, and set kernel and arguments
  int status = 0;
  clgl_buffer_ = clCreateFromGLBuffer(gpu_pro_->context(),
                                      CL_MEM_WRITE_ONLY,
                                      pbuffer_,
                                      &status);
  exp_img_ = new bocl_mem(gpu_pro_->context(), /*(void*) pbuffer_*/ NULL, ni_*nj_*sizeof(GLubyte)*4, "exp image (gl) buffer");
  exp_img_->set_gl_buffer(clgl_buffer_);
  
  //initialize the GPU render process
  vcl_string render_opts = "";
  render_.init_kernel(&gpu_pro_->context(), &gpu_pro_->devices()[0], render_opts);
  //render_.set_image(exp_img_);
  render_.set_gl_image(exp_img_);

  //initlaize gpu update process
  vcl_string update_opts = "";
  update_.init_kernel(&gpu_pro_->context(), &gpu_pro_->devices()[0], update_opts);

  //initialize refine process
  refine_.init_kernel(&gpu_pro_->context(), &gpu_pro_->devices()[0], "");

  return true;
}

//: private helper method to create clgl context using cl_context properties
cl_context boxm2_render_rgb_tableau::create_clgl_context()
{
  //init glew
  GLenum err = glewInit();
  if (GLEW_OK != err)
    vcl_cout<< "GlewInit Error: "<<glewGetErrorString(err)<<vcl_endl;    // Problem: glewInit failed, something is seriously wrong.

  //initialize the render manager
  cl_device_id device = gpu_pro_->devices()[0];
  cl_platform_id platform_id[1];
  int status = clGetDeviceInfo(device,CL_DEVICE_PLATFORM,sizeof(platform_id),(void*) platform_id,NULL);
  if (!check_val(status, CL_SUCCESS, "boxm2_render Tableau::create_cl_gl_context CL_DEVICE_PLATFORM failed."))
    return 0;

  ////create OpenCL context
  cl_context ComputeContext;
#ifdef WIN32
  cl_context_properties props[] =
  {
    CL_GL_CONTEXT_KHR, (cl_context_properties) wglGetCurrentContext(),
    CL_WGL_HDC_KHR, (cl_context_properties) wglGetCurrentDC(),
    CL_CONTEXT_PLATFORM, (cl_context_properties) platform_id[0],
    0
  };
  //create OpenCL context with display properties determined above
  ComputeContext = clCreateContext(props, 1, &device, NULL, NULL, &status);
#elif defined(__APPLE__) || defined(MACOSX)
  CGLContextObj kCGLContext = CGLGetCurrentContext();
  CGLShareGroupObj kCGLShareGroup = CGLGetShareGroup(kCGLContext);

  cl_context_properties props[] = {
    CL_CONTEXT_PROPERTY_USE_CGL_SHAREGROUP_APPLE, (cl_context_properties)kCGLShareGroup,
    CL_CONTEXT_PLATFORM, (cl_context_properties) platform_id[0],
    0
  };
  //create a CL context from a CGL share group - no GPU devices must be passed,
  //all CL compliant devices in the CGL share group will be used to create the context. more info in cl_gl_ext.h
  ComputeContext = clCreateContext(props, 0, 0, NULL, NULL, &status);
#else
  cl_context_properties props[] =
  {
      CL_GL_CONTEXT_KHR, (cl_context_properties)glXGetCurrentContext(),
      CL_GLX_DISPLAY_KHR, (cl_context_properties)glXGetCurrentDisplay(),
      CL_CONTEXT_PLATFORM, (cl_context_properties) platform_id[0],
      0
  };
  ComputeContext = clCreateContext(props, 1, &device, NULL, NULL, &status);
#endif

  if (status!=CL_SUCCESS) {
    vcl_cout<<"Error: Failed to create a compute CL/GL context!" << error_to_string(status) <<vcl_endl;
    return 0;
  }
  return ComputeContext;
}
