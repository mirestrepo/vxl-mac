#include "boxm2_ocl_render_tableau.h"
//:
// \file
#include <vpgl/vpgl_perspective_camera.h>
#include <vgui/vgui_modifier.h>
#include <vcl_sstream.h>
#include <boxm2/ocl/boxm2_ocl_util.h>
#include <boxm2/view/boxm2_view_utils.h>
#include <boxm2/boxm2_util.h>
#include <vcl_algorithm.h>
#include <vil/vil_save.h>


#include <bocl/bocl_device.h>
#include <bocl/bocl_kernel.h>


#include <brdb/brdb_value.h>
#include <brdb/brdb_selection.h>

#include <bprb/bprb_batch_process_manager.h>
#include <bprb/bprb_parameters.h>
#include <bprb/bprb_macros.h>
#include <bprb/bprb_func_process.h>

#include <boxm2/ocl/algo/boxm2_ocl_render_expected_image_function.h>




namespace boxm2_ocl_render_gl_expected_image_process_globals2
{
  const unsigned n_inputs_ = 9 ;
  const unsigned n_outputs_ = 1;
  vcl_size_t lthreads[2]={8,8};
  
  static vcl_map<vcl_string,vcl_vector<bocl_kernel*> > kernels;
  
  void compile_kernel(bocl_device_sptr device,vcl_vector<bocl_kernel*> & vec_kernels, vcl_string opts)
  {
    //gather all render sources... seems like a lot for rendering...
    vcl_vector<vcl_string> src_paths;
    vcl_string source_dir = boxm2_ocl_util::ocl_src_root();
    src_paths.push_back(source_dir + "scene_info.cl");
    src_paths.push_back(source_dir + "bit/bit_tree_library_functions.cl");
    src_paths.push_back(source_dir + "backproject.cl");
    src_paths.push_back(source_dir + "statistics_library_functions.cl");
    src_paths.push_back(source_dir + "expected_functor.cl");
    src_paths.push_back(source_dir + "ray_bundle_library_opt.cl");
    src_paths.push_back(source_dir + "bit/render_bit_scene.cl");
    src_paths.push_back(source_dir + "bit/cast_ray_bit.cl");
    
    //set kernel options
    //#define STEP_CELL step_cell_render(mixture_array, alpha_array, data_ptr, d, &vis, &expected_int);
    
    vcl_string options = opts;
    options += " -D RENDER";
    options += " -D DETERMINISTIC";
    options += " -D STEP_CELL=step_cell_render(aux_args.mog,aux_args.alpha,data_ptr,d*linfo->block_len,vis,aux_args.expint)";
    
    //have kernel construct itself using the context and device
    bocl_kernel * ray_trace_kernel=new bocl_kernel();
    vcl_cout << options << vcl_endl;
    ray_trace_kernel->create_kernel( &device->context(),
                                    device->device_id(),
                                    src_paths,
                                    "render_bit_scene",   //kernel name
                                    options,              //options
                                    "boxm2 opencl render"); //kernel identifier (for error checking)
    vec_kernels.push_back(ray_trace_kernel);
    //create normalize image kernel
    vcl_vector<vcl_string> norm_src_paths;
    norm_src_paths.push_back(source_dir + "pixel_conversion.cl");
    norm_src_paths.push_back(source_dir + "bit/normalize_kernels.cl");
    bocl_kernel * normalize_render_kernel=new bocl_kernel();
    
    normalize_render_kernel->create_kernel( &device->context(),
                                           device->device_id(),
                                           norm_src_paths,
                                           "normalize_render_kernel",   //kernel name
                                           "-D RENDER",              //options
                                           "normalize render kernel gl"); //kernel identifier (for error checking)
    
    vec_kernels.push_back(normalize_render_kernel);
  }
}



//: Constructor
boxm2_ocl_render_tableau::boxm2_ocl_render_tableau()
{
  is_bw_ = true;
  pbuffer_=0;
  ni_=640;
  nj_=480;
}

//: initialize tableau properties
bool boxm2_ocl_render_tableau::init(bocl_device_sptr device,
                                    boxm2_opencl_cache_sptr opencl_cache,
                                    boxm2_scene_sptr scene,
                                    unsigned ni,
                                    unsigned nj,
                                    vpgl_perspective_camera<double> * cam, vcl_string identifier)
{
  //set image dimensions, camera and scene
  ni_ = ni;
  nj_ = nj;
  
  identifier_ = identifier;
  
  cam_   = (*cam);
  default_cam_ = (*cam);
  
  default_stare_point_.set(scene->bounding_box().centroid().x(),
                           scene->bounding_box().centroid().y(),
                           scene->bounding_box().min_z());
  stare_point_=default_stare_point_;
  scale_ =scene->bounding_box().height();
  //create the scene
  scene_ = scene;
  opencl_cache_=opencl_cache;
  device_=device;
  do_init_ocl=true;
  render_trajectory_ = true;
  trajectory_ = new boxm2_trajectory(30.0, 45.0, -1.0, scene_->bounding_box(), ni, nj);
  cam_iter_ = trajectory_->begin();
  return true;
}


//: Handles tableau events (drawing and keys)
bool boxm2_ocl_render_tableau::handle(vgui_event const &e)
{
  //draw handler - called on post_draw()
  if (e.type == vgui_DRAW)
  {
    if (do_init_ocl) {
      this->init_clgl();
      do_init_ocl = false;
    }
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
    str<<".  rendering at ~ "<< (1000.0f / gpu_time) <<" fps ";
    status_->write(str.str().c_str());
    
    return true;
  }
  
  
  //toggle color - this is a hack to get color models to show as grey
  if (e.type == vgui_KEY_PRESS) {
    if (e.key == vgui_key('c')) {
      vcl_cout<<"Toggling b and w"<<vcl_endl;
      is_bw_ = !is_bw_;
    }
  }
  
  if (boxm2_cam_tableau::handle(e)) {
    return true;
  }
  
  return false;
}

//: calls on ray manager to render frame into the pbuffer_
float boxm2_ocl_render_tableau::render_frame()
{
  
  using namespace boxm2_ocl_render_gl_expected_image_process_globals2;
  
  cl_int status = clEnqueueAcquireGLObjects( queue_, 1,&exp_img_->buffer(), 0, 0, 0);
  //exp_img_->zero_gpu_buffer( queue_ );
  if (!check_val(status,CL_SUCCESS,"clEnqueueAcquireGLObjects failed. (gl_image)"+error_to_string(status)))
    return -1.0f;
  
  vcl_cout<<cam_<<vcl_endl;
  ////////////
  //get scene data type and appTypeSize
  vcl_string data_type;
  int apptypesize;
  vcl_vector<vcl_string> valid_types;
  valid_types.push_back(boxm2_data_traits<BOXM2_MOG3_GREY>::prefix());
  valid_types.push_back(boxm2_data_traits<BOXM2_GAUSS_GREY>::prefix());
  valid_types.push_back(boxm2_data_traits<BOXM2_MOG3_GREY_16>::prefix());
  if( !boxm2_util::verify_appearance( *scene_, valid_types, data_type, apptypesize ) ) {
    vcl_cout<<"boxm2_ocl_paint_batch ERROR: scene doesn't have BOXM2_MOG3_GREY or BOXM2_MOG3_GREY_16 data type"<<vcl_endl;
    return false;
  }
  
  //make sure apperance identifier is correct
  vcl_string app_identifier = identifier_;
  if (app_identifier.size() > 0)
    data_type += "_" + app_identifier;
  
  //get initial options (MOG TYPE)
  vcl_string options = boxm2_ocl_util::mog_options(data_type);
  
  //: create a command queue.
  status=0;
  queue_ = clCreateCommandQueue(device_->context(),*(device_->device_id()),
                                CL_QUEUE_PROFILING_ENABLE,&status);
  if (status!=0) return false;
  vcl_string identifier=device_->device_identifier()+options;
  
  // compile the kernel
  if (kernels.find(identifier)==kernels.end())
  {
    vcl_cout<<"===========Compiling kernels==========="<<vcl_endl;
    vcl_vector<bocl_kernel*> ks;
    compile_kernel(device_,ks,options);
    kernels[identifier]=ks;
  }
  
  unsigned cl_ni=RoundUp(ni_,8);
  unsigned cl_nj=RoundUp(nj_,8);
  
  //expected image buffer
  float* buff = new float[cl_ni*cl_nj];
  for (unsigned i=0;i<cl_ni*cl_nj;i++) buff[i]=0.0f;
  bocl_mem_sptr exp_image=new bocl_mem(device_->context(), buff, cl_ni*cl_nj*sizeof(float), "exp image buffer");
  exp_image->create_buffer(CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);
  
  //dimensions
  int img_dim_buff[4];
  img_dim_buff[0] = 0;   img_dim_buff[2] = ni_;
  img_dim_buff[1] = 0;   img_dim_buff[3] = nj_;
  exp_img_dim_=new bocl_mem(device_->context(), img_dim_buff, sizeof(int)*4, "image dims");
  exp_img_dim_->create_buffer(CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);
  
  // visibility image
  float* vis_buff = new float[cl_ni*cl_nj];
  vcl_fill(vis_buff, vis_buff + cl_ni*cl_nj, 1.0f);
  bocl_mem_sptr vis_image = new bocl_mem(device_->context(), vis_buff, cl_ni*cl_nj*sizeof(float), "vis image buffer");
  vcl_cout << "Size of vis buffer: " << cl_ni*cl_nj*sizeof(float) << vcl_endl;
  vis_image->create_buffer(CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);
  float* max_omega_buff = new float[cl_ni*cl_nj];
  vcl_fill(max_omega_buff, max_omega_buff + cl_ni*cl_nj, 0.0f);
  bocl_mem_sptr max_omega_image =new bocl_mem(device_->context(), max_omega_buff, cl_ni*cl_nj*sizeof(float), "omega image buffer");
  //bocl_mem_sptr max_omega_image = opencl_cache->alloc_mem(cl_ni*cl_nj*sizeof(float), max_omega_buff,"vis image buffer");
  max_omega_image->create_buffer(CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);

  
  // run expected image function
  vpgl_camera_double_sptr cam = new vpgl_perspective_camera<double>(cam_);
  vcl_size_t lthreads[2]={8,8};
  float time = render_expected_image( scene_, device_, opencl_cache_, queue_,
                                     cam, exp_image, vis_image, max_omega_image, exp_img_dim_,
                                     data_type, kernels[identifier][0], lthreads, cl_ni, cl_nj, apptypesize);
  
  // normalize
  {
    vcl_size_t gThreads[] = {cl_ni,cl_nj};
    bocl_kernel* normalize_kern= kernels[identifier][1];
    normalize_kern->set_arg( exp_image.ptr() );
    normalize_kern->set_arg( vis_image.ptr() );
    normalize_kern->set_arg( exp_img_dim_.ptr());
    normalize_kern->execute( queue_, 2, lthreads, gThreads);
    clFinish(queue_);
    
    //clear render kernel args so it can reset em on next execution
    normalize_kern->clear_args();
    time += normalize_kern->exec_time();
  }
  // read out expected image
  exp_image->read_to_buffer(queue_);
  clReleaseCommandQueue(queue_);
  
  
  vil_image_view<float>* exp_img_out=new vil_image_view<float>(ni_,nj_);
  for (unsigned c=0;c<nj_;c++)
    for (unsigned r=0;r<ni_;r++)
      (*exp_img_out)(r,c)= (buff[c*cl_ni+r] );
  
  vcl_cout << "Saving expected image \n";
  vil_save(*exp_img_out, "./test_buff.tiff");
  
  
  delete [] vis_buff;
  delete [] buff;
  opencl_cache_->unref_mem(vis_image.ptr());
  opencl_cache_->unref_mem(exp_image.ptr());
  
  //release gl buffer
  status = clEnqueueReleaseGLObjects(queue_, 1, &clgl_buffer_, 0, 0, 0);
  clFinish( queue_ );
  
  return time;
}

//: private helper method to init_clgl stuff (gpu processor)
bool boxm2_ocl_render_tableau::init_clgl()
{
  //get relevant blocks
  vcl_cout<<"Data Path: "<<scene_->data_path()<<vcl_endl;
  device_->context() = boxm2_view_utils::create_clgl_context(*(device_->device_id()));
  opencl_cache_->set_context(device_->context());
  
  int status_queue=0;
  queue_ =  clCreateCommandQueue(device_->context(),*(device_->device_id()),CL_QUEUE_PROFILING_ENABLE,&status_queue);
  
  if (!check_val(status_queue,CL_SUCCESS,"boxm2_ocl_render_tableau::init_clgl failed to create cl command queue " + error_to_string(status_queue)))
    return false;
  
  
  // delete old buffer
  if (pbuffer_) {
    clReleaseMemObject(clgl_buffer_);
    glDeleteBuffers(1, &pbuffer_);
  }
  
  
  ////generate glBuffer, and bind to ray_mgr->image_gl_buf_
  glGenBuffers(1, &pbuffer_);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbuffer_);
  glBufferData(GL_PIXEL_UNPACK_BUFFER, ni_*nj_*sizeof(GLubyte)*4, NULL, GL_STREAM_DRAW);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
  
  
  //create OpenCL buffer from GL PBO, and set kernel and arguments
  int status = 0;
  clgl_buffer_ = clCreateFromGLBuffer(device_->context(),
                                      CL_MEM_READ_WRITE,
                                      pbuffer_,
                                      &status);
  
  if (!check_val(status,CL_SUCCESS,"boxm2_ocl_render_tableau::init_clgl failed to create gl buffer " + error_to_string(status)))
    return false;
  
  exp_img_ = new bocl_mem(device_->context(),  NULL, RoundUp(ni_,8)*RoundUp(nj_,8)*sizeof(GLubyte), "exp image (gl) buffer");
  exp_img_->set_gl_buffer(clgl_buffer_);
  
  int img_dim_buff[4];
  img_dim_buff[0] = 0;   img_dim_buff[2] = ni_;
  img_dim_buff[1] = 0;   img_dim_buff[3] = nj_;
  exp_img_dim_=new bocl_mem(device_->context(), img_dim_buff, sizeof(int)*4, "image dims");
  exp_img_dim_->create_buffer(CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);
  return true;
}
