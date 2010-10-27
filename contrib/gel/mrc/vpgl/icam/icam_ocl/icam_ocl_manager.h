// This is gel/mrc/vpgl/icam/icam_ocl/icam_ocl_manager.h
#ifndef icam_ocl_manager_h_
#define icam_ocl_manager_h_
//:
// \file
// \brief
//  A parent class for singleton opencl managers
// \author J. Mundy
// \date November 13, 2009
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <vcl_vector.h>
#include <vcl_map.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include "icam_ocl_cl.h"
#include <vcl_cstddef.h>
#if !defined(__APPLE__)
#include <malloc.h>
#endif
#define SDK_SUCCESS 0
#define SDK_FAILURE 1
//#define GROUP_SIZE 64
#define VECTOR_SIZE 4

template <class T>
class icam_ocl_manager
{
 protected:
  vcl_size_t number_devices_;
  vcl_size_t max_work_group_size_;   //!< Max allowed work-items in a group
  cl_uint max_dimensions_;           //!< Max group dimensions allowed
  vcl_size_t * max_work_item_sizes_; //!< Max work-items sizes in each dimension
  cl_ulong total_local_memory_;      //!< Max local memory allowed
  cl_ulong total_global_memory_;     //!< Max global memory allowed
  cl_uint max_compute_units_;        //!< Max compute units
  cl_uint vector_width_short_;       //!< Ideal short vector size
  cl_uint vector_width_float_;       //!< Ideal float vector size
  cl_uint max_clock_freq_;           //!< Maximum clock frequency
  cl_bool image_support_;            //!< image support
  cl_device_id *devices_;            //!< CL device list
  vcl_size_t image2d_max_width_;     //!< Ideal image dimensions
  vcl_size_t image2d_max_height_;
  cl_char extensions_supported_[1000]; //! Support character addressing
  char platform_name[100];
 public:
  cl_context context_;               //!< CL context

  //: Destructor
  virtual ~icam_ocl_manager();

  //: Use this instead of constructor
  static T* instance();

  //: Initialise the opencl environment
  void clear_cl();

  //: Initialise the opencl environment
  bool initialize_cl();

  //: Check for error returns
  int check_val(cl_int status, cl_int result, std::string message) {
    if (status != result) {
      vcl_cout << message << '\n';
      return 0;
    }
    return 1;
  }

  vcl_size_t group_size() const {return max_work_group_size_;}
  cl_ulong total_local_memory() const {return total_local_memory_;}
  cl_context context() {return context_;}
  cl_device_id * devices() {return devices_;}

  //: Allocate host memory for use with clCreateBuffer (aligned if necessary)
  void* allocate_host_mem(vcl_size_t size);
  bool load_kernel_source(vcl_string const& path);
  bool append_process_kernels(vcl_string const& path);
  bool write_program(vcl_string const& path);
  vcl_string program_source() const {return prog_;}

  //build kernel program:
  int build_kernel_program(cl_program & program, vcl_string options);

  cl_bool image_support(){return image_support_;}

 protected:

  //: Constructor
  icam_ocl_manager() : max_work_item_sizes_(0), devices_(0) {}

  static T* instance_;

  vcl_string prog_;
};

#endif // icam_ocl_manager_h_