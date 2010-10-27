// This is vpgl/icam/icam_ocl/icam_ocl_manager.txx
#ifndef icam_ocl_manager_txx_
#define icam_ocl_manager_txx_

#include "icam_ocl_manager.h"
//:
// \file

#include <vcl_utility.h>
#include <vcl_fstream.h>
#include <vcl_sstream.h>
#include <icam_ocl/icam_ocl_utils.h>
#include <vcl_cstdio.h>


//: Insure only one instance is created
template <class T>
T* icam_ocl_manager<T>::instance()
{
  if (!instance_) {
    instance_ = new T();
    instance_->initialize_cl();
  }
  return icam_ocl_manager::instance_;
}

template <class T>
void icam_ocl_manager<T>::clear_cl()
{
  clReleaseContext(context_);
  if (devices_)
  {
    free(devices_);
    devices_ = NULL;
  }

  if (max_work_item_sizes_)
  {
    free(max_work_item_sizes_);
    max_work_item_sizes_ = NULL;
  }
}

//: Destructor
template <class T>
icam_ocl_manager<T>::~icam_ocl_manager()
{
  this->clear_cl();
}


template <class T>
bool icam_ocl_manager<T>::initialize_cl()
{
  cl_int status = CL_SUCCESS;
  cl_uint num_platforms = 0;
  // Check the number of  available platforms
  status = clGetPlatformIDs(0,NULL,&num_platforms);
  if (status != CL_SUCCESS) {
    vcl_cerr << "icam_ocl_manager: clGetPlatformIDs (call 1) returned " << status << '\n';
    return false;
  }
  if (num_platforms == 0) {
    vcl_cerr << "icam_ocl_manager: 0 OpenCL platforms found!\n";
    return false;
  }
  if (num_platforms > 1) {
    vcl_cerr << "icam_ocl_manager: warning: found " << num_platforms << "OpenCL platforms. Using the first\n";
  }
  // Get the first platform ID
  cl_platform_id platform_id[2];
  status = clGetPlatformIDs (num_platforms, platform_id, NULL);
  if (status != CL_SUCCESS) {
    vcl_cerr << "icam_ocl_manager: clGetPlatformIDs (call 2) returned " << status << '\n';
    return false;
  }

  vcl_size_t ret_size;
  
  bool gpu_found=false;
  bool cpu_found=false;

  cl_device_id device;
  cl_device_id gpus[2];
  cl_uint numGPUs;
  //: First checking for GPU
  for (unsigned i=0;i<num_platforms;i++)
  {

    if ( clGetDeviceIDs(platform_id[i], CL_DEVICE_TYPE_GPU, 2, gpus, &numGPUs)== CL_SUCCESS)
    {
        clGetPlatformInfo(platform_id[i],CL_PLATFORM_NAME,sizeof(platform_name),platform_name,&ret_size);

      gpu_found=true;
      vcl_cout<<"Found "<<numGPUs<<" GPUs"<<vcl_endl;
      //use the second GPU if it's there...
      device = (numGPUs > 1)? gpus[1] : gpus[0];
      //device = gpus[0]; 
      break;
    }
  }
  //: If GPU not found then look for CPU
  if (!gpu_found)
  {
    for (unsigned i=0;i<num_platforms;i++)
    {
      if ( clGetDeviceIDs(platform_id[i], CL_DEVICE_TYPE_CPU, 1, &device, NULL)== CL_SUCCESS)
      {
        cpu_found=true;
        break;
      }
    }
  }
  // get an available GPU device from the the platform
  // should we be using all if more than one available?

  if (!gpu_found && !cpu_found)
    return false;

  //Create a context from the device ID
  context_ = clCreateContext(0, 1, &device, NULL, NULL, &status);
  if (!this->check_val(status,CL_SUCCESS,"clCreateContextFromType failed.")) {
    return false;
  }

  vcl_size_t device_list_size = 0;
  // First, get the size of device list data
  status = clGetContextInfo(context_,
                            CL_CONTEXT_DEVICES,
                            0,
                            NULL,
                            &device_list_size);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetContextInfo failed."))
    return false;
  
  number_devices_ = device_list_size/sizeof(cl_device_id);

  // Now allocate memory for device list based on the size we got earlier
  devices_ = (cl_device_id *)malloc(device_list_size);
  if (devices_==NULL) {
    vcl_cout << "Failed to allocate memory (devices).\n";
    return false;
  }

  // Now, get the device list data
  status = clGetContextInfo(context_,
                            CL_CONTEXT_DEVICES,
                            device_list_size,
                            devices_,
                            NULL);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetGetContextInfo failed."))
    return false;

  vcl_size_t max_work_group_size = 0;
  
  // Get device specific information
  char vendor[512];
  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_VENDOR,
                           sizeof(vendor),
                           (void*) vendor,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_VENDOR failed."))
    return false;


  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_MAX_WORK_GROUP_SIZE,
                           sizeof(vcl_size_t),
                           (void*)&max_work_group_size,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_MAX_WORK_GROUP_SIZE failed."))
    return false;

  max_work_group_size_ =max_work_group_size/sizeof(vcl_size_t);

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                           sizeof(cl_uint),
                           (void*)&max_dimensions_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS failed."))
    return false;


  max_work_item_sizes_ = (vcl_size_t*)malloc(max_dimensions_ * sizeof(vcl_size_t));

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_MAX_WORK_ITEM_SIZES,
                           sizeof(vcl_size_t) * max_dimensions_,
                           (void*)max_work_item_sizes_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_MAX_WORK_ITEM_SIZES failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_LOCAL_MEM_SIZE,
                           sizeof(cl_ulong),
                           (void *)&total_local_memory_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_LOCAL_MEM_SIZE failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_GLOBAL_MEM_SIZE,
                           sizeof(cl_ulong),
                           (void *)&total_global_memory_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_GLOBAL_MEM_SIZE failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_MAX_COMPUTE_UNITS,
                           sizeof(cl_uint),
                           (void *)&max_compute_units_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_MAX_COMPUTE_UNITS failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                           sizeof(cl_uint),
                           (void *)&vector_width_short_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                           sizeof(cl_uint),
                           (void *)&vector_width_float_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_MAX_CLOCK_FREQUENCY,
                           sizeof(cl_uint),
                           (void *)&max_clock_freq_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_MAX_CLOCK_FREQUENCY failed."))
    return false;

  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_IMAGE_SUPPORT,
                           sizeof(cl_bool),
                           (void *)&image_support_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_IMAGE_SUPPORT failed."))
    return false;
  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_IMAGE2D_MAX_WIDTH,
                           sizeof(vcl_size_t),
                           (void *)&image2d_max_width_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_IMAGE_SUPPORT failed."))
    return false;
  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_IMAGE2D_MAX_HEIGHT,
                           sizeof(vcl_size_t),
                           (void *)&image2d_max_height_,
                           NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_IMAGE_SUPPORT failed."))
            return false;
 
  char extensions[512];
  status = clGetDeviceInfo(devices_[0],
                           CL_DEVICE_EXTENSIONS,
                           sizeof(extensions),
                           (void*) extensions,
                           NULL);
  
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetDeviceInfo CL_DEVICE_IMAGE_SUPPORT failed."))
    return false;

  unsigned size = sizeof(vcl_size_t);
  vcl_cout << " Context Description\n"
           << " Platform Name: "<<platform_name <<'\n'
           << " Device vendor: " << vendor << '\n' 
           << " Device extensions: " << extensions << 'n'
           << " Number of devices: " << number_devices_ << '\n'
           << " Number of compute units: " << max_compute_units_ << '\n'
           << " Maximum clock frequency: " << max_clock_freq_/1000.0 << " GHz\n"
           << " Total global memory: "<<total_global_memory_/ 1073741824.0 /* 2^30 */ << " GBytes\n"
           << " Total local memory: "<< total_local_memory_/1024.0 << " KBytes\n"
           << " Maximum work group size: " << max_work_group_size_ << '\n'
           << " Maximum work item sizes: (" << (cl_uint)max_work_item_sizes_[0]/size << ','
           << (cl_uint)max_work_item_sizes_[1]/size << ','
           << (cl_uint)max_work_item_sizes_[2]/size << ")\n"
           << " Preferred short vector length: " << vector_width_short_ << '\n'
           << " Preferred float vector length: " << vector_width_float_ << '\n'
           << " image support " << image_support_ << '\n'
           << " Max 2D image width  " << image2d_max_width_ << '\n'
           << " Max 2D image height  " << image2d_max_height_ << '\n'
  ;
  for (unsigned id = 0; id<number_devices_; ++id)
    vcl_cout << " Device id [" << id << "]: " << devices_[id] << '\n';
  return true;
}

template<class T>
bool icam_ocl_manager<T>::load_kernel_source(vcl_string const& path)
{
  prog_ = "";
  vcl_ifstream is(path.c_str());
  if (!is.is_open())
    return false;
  char temp[256];
  vcl_ostringstream ostr;
  while (!is.eof()) {
    is.getline(temp, 256);
    vcl_string s(temp);
    ostr << s << '\n';
  }
  prog_ =  ostr.str();
  return prog_.size() > 0;
}

template<class T>
bool icam_ocl_manager<T>::append_process_kernels(vcl_string const& path)
{
  vcl_ifstream is(path.c_str());
  if (!is.is_open())
    return false;
  char temp[256];
  vcl_ostringstream ostr;
  while (!is.eof()) {
    is.getline(temp, 256);
    vcl_string s(temp);
    ostr << s << '\n';
  }
  prog_ += ostr.str();
  return true;
}

template<class T>
bool icam_ocl_manager<T>::write_program(vcl_string const& path)
{
  vcl_ofstream os(path.c_str());
  if (!os.is_open())
    return false;
  os << prog_;
  return true;
}

template <class T>
void* icam_ocl_manager<T>::allocate_host_mem(vcl_size_t size)
{
#if defined (_WIN32)
  return _aligned_malloc(size, 16);
#elif defined(__APPLE__)
  return malloc(size);
#else
  return memalign(16, size);
#endif
}

template<class T>
int icam_ocl_manager<T>::build_kernel_program(cl_program & program, vcl_string options)
{
  cl_int status = CL_SUCCESS;
  vcl_size_t sourceSize[] = { this->prog_.size() };
  if (!sourceSize[0]) return SDK_FAILURE;
  if (program) {
    status = clReleaseProgram(program);
    program = 0;
    if (!this->check_val(status, CL_SUCCESS, "clReleaseProgram failed."))
      return SDK_FAILURE;
  }
  const char * source = this->prog_.c_str();

  program = clCreateProgramWithSource(this->context_, 
                                      1, 
                                      &source,  
                                      sourceSize,
                                      &status);
  if (!this->check_val(status,CL_SUCCESS,"clCreateProgramWithSource failed."))
    return SDK_FAILURE;

  // create a cl program executable for all the devices specified
  status = clBuildProgram(program,
                          1,
                          this->devices_,
                          options.c_str(),
                          NULL,
                          NULL);
  if (!this->check_val(status, CL_SUCCESS, error_to_string(status)))
  {
    vcl_size_t len;
    char buffer[2048];
    clGetProgramBuildInfo(program, this->devices_[0],
                          CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
    vcl_printf("%s\n", buffer);
    return SDK_FAILURE;
  }
  else
    return SDK_SUCCESS;
}


#undef ICAM_OCL_MANAGER_INSTANTIATE
#define ICAM_OCL_MANAGER_INSTANTIATE(T) \
template <class T > T* icam_ocl_manager<T >::instance_ = 0; \
template class icam_ocl_manager<T >


#endif // icam_ocl_manager_txx_