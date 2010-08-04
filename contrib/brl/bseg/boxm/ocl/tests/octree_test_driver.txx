#ifndef octree_test_driver_txx_
#define octree_test_driver_txx_

#include "octree_test_driver.h"

#include <vcl_fstream.h>
#include <vcl_iomanip.h>
#include <vcl_sstream.h>
#include <vcl_ctime.h>
#include <vcl_cmath.h>
#include <vcl_cstdlib.h>
#include <vcl_cstdio.h>
#include <vcl_cassert.h>

template <class T>
int
octree_test_driver<T>::setup_cl()
{
  // Create command queue
  cl_int status = CL_SUCCESS;

  cl_device_id* device_p = cl_manager_->devices();
  if (device_p) {
    command_queue_ = clCreateCommandQueue(cl_manager_->context(),
                                          device_p[0],
                                          CL_QUEUE_PROFILING_ENABLE,
                                          &status);
  }
  else {
    return SDK_FAILURE;
  }

  if (!this->check_val(status,
    CL_SUCCESS,
    "clCreateCommandQueue failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
void octree_test_driver<T>::set_buffers()
{
  input_cell_buf_ =cl_manager_->cell_buf();
  input_data_buf_ = cl_manager_->data_buf();
  input_ray_origin_buf_ = cl_manager_->ray_origin_buf();
  //input_ray_dir_buf_ = cl_manager_->ray_dir_buf();
  input_camera_buf_  =cl_manager_->camera_buf();
  input_imgdims_buf_ =cl_manager_->imgdims_buf();
}

template <class T>
int octree_test_driver<T>::create_kernel(vcl_string const& kernel_name)
{
  if (cl_manager_->create_kernel(kernel_name)!=SDK_SUCCESS)
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::set_tree_args()
{
  cl_int   status;
  // -- Set appropriate arguments to the kernel --
  // the array of tree cells
  status = clSetKernelArg(cl_manager_->kernel(),
                          0,
                          sizeof(cl_mem),
                          (void *)&input_cell_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (input_cell_array)"))
    return SDK_FAILURE;
  // the array of tree data
  status = clSetKernelArg(cl_manager_->kernel(),
                          1,
                          sizeof(cl_mem),
                          (void *)&input_data_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (input_data_array)"))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::set_ray_trace_args()
{
  cl_int   status;
  // -- Set appropriate arguments to the kernel --
  // the ray origin buffer
  status = clSetKernelArg(cl_manager_->kernel(),
                          2,
                          sizeof(cl_mem),
                          (void *)&input_ray_origin_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (ray_origin_array)"))
    return SDK_FAILURE;
  // the ray direction buffer
  status = clSetKernelArg(cl_manager_->kernel(),
                          3,
                          sizeof(cl_mem),
                          (void *)&input_ray_dir_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (ray_dir_array)"))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::set_camera_args()
{
  cl_int   status;
  // -- Set appropriate arguments to the kernel --
  // the array of tree cells
  status = clSetKernelArg(cl_manager_->kernel(),
                          0,
                          sizeof(cl_mem),
                          (void *)&input_camera_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (input_cell_array)"))
    return SDK_FAILURE;
  // the array of tree data
  status = clSetKernelArg(cl_manager_->kernel(),
                          1,
                          sizeof(cl_mem),
                          (void *)&input_imgdims_buf_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (input_data_array)"))
    return SDK_FAILURE;
  status = clSetKernelArg(cl_manager_->kernel(),
                          2,
                          sizeof(cl_float16)*3,
                          0);
  if (!this->check_val(status,
      CL_SUCCESS,
      "clSetKernelArg failed. (input_data_array)"))
      return SDK_FAILURE;

  else
    return SDK_SUCCESS;
}


template <class T>
int octree_test_driver<T>::run_image_tree_test_kernels()
{
  cl_int   status;
  cl_event events[2];

  //// clear output array
  cl_manager_->clear_tree_results();

  cl_mem result_buf = clCreateBuffer(cl_manager_->context(),
                                     CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                                     cl_manager_->tree_result_size() * sizeof(cl_int4),
                                     cl_manager_->tree_results(),
                                     &status);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clCreateBuffer failed. (tree_results)"))
    return SDK_FAILURE;
  else if (this->set_tree_args()!=SDK_SUCCESS)
    return SDK_FAILURE;

  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          2,
                          sizeof(cl_mem),
                          (void *)&result_buf);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;
  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_LOCAL_MEM_SIZE,
                                    sizeof(cl_ulong),
                                    &used_local_memory_,
                                    NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_LOCAL_MEM_SIZE failed."))
  {
    return SDK_FAILURE;
  }

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_WORK_GROUP_SIZE,
                                    sizeof(cl_ulong),
                                    &kernel_work_group_size_,
                                    NULL);
  if (!this->check_val(status,
                      CL_SUCCESS,
                      "clGetKernelWorkGroupInfo CL_KERNEL_WORK_GROUP_SIZE, failed."))
  {
    return SDK_FAILURE;
  }

  vcl_size_t globalThreads[]= {cl_manager_->n_ray_groups()*
                               cl_manager_->group_size()};
  vcl_size_t localThreads[] = {cl_manager_->group_size()};

  if (used_local_memory_ > cl_manager_->total_local_memory())
  {
    vcl_cout << "Unsupported: Insufficient local memory on device.\n";
    return SDK_FAILURE;
  }
  //  vcl_cout << "Local memory used: " << usedLocalMemory << '\n';

  status = clEnqueueNDRangeKernel(command_queue_,
                                  cl_manager_->kernel(),
                                  1,
                                  NULL,
                                  globalThreads,
                                  localThreads,
                                  0,
                                  NULL,
                                  NULL);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueNDRangeKernel failed."))
    return SDK_FAILURE;

  status = clFinish(command_queue_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clFinish failed."))
    return SDK_FAILURE;

  // Enqueue readBuffers
  status = clEnqueueReadBuffer(command_queue_,
                               result_buf,
                               CL_TRUE,
                               0,
                               cl_manager_->tree_result_size()*sizeof(cl_int4),
                               cl_manager_->tree_results(),
                               0,
                               NULL,
                               &events[0]);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueBuffer (tree_results)failed."))
    return SDK_FAILURE;

  // Wait for the read buffer to finish execution
  status = clWaitForEvents(1, &events[0]);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clWaitForEvents failed."))
    return SDK_FAILURE;

  clReleaseEvent(events[0]);

  status = clReleaseMemObject(result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseMemObject failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}


template <class T>
int octree_test_driver<T>::run_tree_test_kernels()
{
  cl_int   status;
  cl_event events[2];

  //// clear output array
  cl_manager_->clear_tree_results();

  cl_mem result_buf = clCreateBuffer(cl_manager_->context(),
                                     CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                                     cl_manager_->tree_result_size() * sizeof(cl_int4),
                                     cl_manager_->tree_results(),
                                     &status);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clCreateBuffer failed. (tree_results)"))
    return SDK_FAILURE;
  else if (this->set_tree_args()!=SDK_SUCCESS)
    return SDK_FAILURE;

  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          2,
                          sizeof(cl_mem),
                          (void *)&result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_LOCAL_MEM_SIZE,
                                    sizeof(cl_ulong),
                                    &used_local_memory_,
                                    NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_LOCAL_MEM_SIZE failed."))
  {
    return SDK_FAILURE;
  }

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_WORK_GROUP_SIZE,
                                    sizeof(cl_ulong),
                                    &kernel_work_group_size_,
                                    NULL);
  if (!this->check_val(status,
                      CL_SUCCESS,
                      "clGetKernelWorkGroupInfo CL_KERNEL_WORK_GROUP_SIZE, failed."))
  {
    return SDK_FAILURE;
  }

  vcl_size_t globalThreads[]= {cl_manager_->n_ray_groups()*
                               cl_manager_->group_size()};
  vcl_size_t localThreads[] = {cl_manager_->group_size()};

  if (used_local_memory_ > cl_manager_->total_local_memory())
  {
    vcl_cout << "Unsupported: Insufficient local memory on device.\n";
    return SDK_FAILURE;
  }
  //  vcl_cout << "Local memory used: " << usedLocalMemory << '\n';
  cl_event ceEvent;

  status = clEnqueueNDRangeKernel(command_queue_,
                                  cl_manager_->kernel(),
                                  1,
                                  NULL,
                                  globalThreads,
                                  localThreads,
                                  0,
                                  NULL,
                                  &  ceEvent);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueNDRangeKernel failed."))
    return SDK_FAILURE;

  status = clFinish(command_queue_);
  cl_ulong tstart,tend;
  status = clGetEventProfilingInfo(ceEvent,CL_PROFILING_COMMAND_END,sizeof(cl_ulong),&tend,0);
  status = clGetEventProfilingInfo(ceEvent,CL_PROFILING_COMMAND_START,sizeof(cl_ulong),&tstart,0);
  gpu_time_= (double)1.0e-6 * (tend - tstart); // convert nanoseconds to milliseconds 

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clFinish failed."))
    return SDK_FAILURE;

  // Enqueue readBuffers
  status = clEnqueueReadBuffer(command_queue_,
                               result_buf,
                               CL_TRUE,
                               0,
                               cl_manager_->tree_result_size()*sizeof(cl_int4),
                               cl_manager_->tree_results(),
                               0,
                               NULL,
                               &events[0]);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueBuffer (tree_results)failed."))
    return SDK_FAILURE;

  // Wait for the read buffer to finish execution
  status = clWaitForEvents(1, &events[0]);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clWaitForEvents failed."))
    return SDK_FAILURE;

  clReleaseEvent(events[0]);

  status = clReleaseMemObject(result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseMemObject failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}
template <class T>
int octree_test_driver<T>::run_tree_with_local_test_kernels()
{
  cl_int   status;
  cl_event events[2];

  //// clear output array
  cl_manager_->clear_tree_results();

  cl_mem result_buf = clCreateBuffer(cl_manager_->context(),
                                     CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                                     cl_manager_->tree_result_size() * sizeof(cl_int4),
                                     cl_manager_->tree_results(),
                                     &status);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clCreateBuffer failed. (tree_results)"))
    return SDK_FAILURE;
  else if (this->set_tree_args()!=SDK_SUCCESS)
    return SDK_FAILURE;

  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          2,
                          sizeof(cl_mem),
                          (void *)&result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;
  status = clSetKernelArg(cl_manager_->kernel(),
                          3,
                          sizeof(cl_int4)*73/*hardcoded for now*/,
                          0);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_LOCAL_MEM_SIZE,
                                    sizeof(cl_ulong),
                                    &used_local_memory_,
                                    NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_LOCAL_MEM_SIZE failed."))
  {
    return SDK_FAILURE;
  }

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_WORK_GROUP_SIZE,
                                    sizeof(cl_ulong),
                                    &kernel_work_group_size_,
                                    NULL);
  if (!this->check_val(status,
                      CL_SUCCESS,
                      "clGetKernelWorkGroupInfo CL_KERNEL_WORK_GROUP_SIZE, failed."))
  {
    return SDK_FAILURE;
  }

  vcl_size_t globalThreads[]= {cl_manager_->n_ray_groups()*
                               cl_manager_->group_size()};
  vcl_size_t localThreads[] = {cl_manager_->group_size()};

  if (used_local_memory_ > cl_manager_->total_local_memory())
  {
    vcl_cout << "Unsupported: Insufficient local memory on device.\n";
    return SDK_FAILURE;
  }
  //  vcl_cout << "Local memory used: " << usedLocalMemory << '\n';
  cl_event ceEvent;

  status = clEnqueueNDRangeKernel(command_queue_,
                                  cl_manager_->kernel(),
                                  1,
                                  NULL,
                                  globalThreads,
                                  localThreads,
                                  0,
                                  NULL,
                                  &  ceEvent);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueNDRangeKernel failed."))
    return SDK_FAILURE;

  status = clFinish(command_queue_);
  cl_ulong tstart,tend;
  status = clGetEventProfilingInfo(ceEvent,CL_PROFILING_COMMAND_END,sizeof(cl_ulong),&tend,0);
  status = clGetEventProfilingInfo(ceEvent,CL_PROFILING_COMMAND_START,sizeof(cl_ulong),&tstart,0);
  gpu_time_= (double)1.0e-6 * (tend - tstart); // convert nanoseconds to milliseconds 

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clFinish failed."))
    return SDK_FAILURE;

  // Enqueue readBuffers
  status = clEnqueueReadBuffer(command_queue_,
                               result_buf,
                               CL_TRUE,
                               0,
                               cl_manager_->tree_result_size()*sizeof(cl_int4),
                               cl_manager_->tree_results(),
                               0,
                               NULL,
                               &events[0]);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueBuffer (tree_results)failed."))
    return SDK_FAILURE;

  // Wait for the read buffer to finish execution
  status = clWaitForEvents(1, &events[0]);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clWaitForEvents failed."))
    return SDK_FAILURE;

  clReleaseEvent(events[0]);

  status = clReleaseMemObject(result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseMemObject failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::run_ray_trace_test_kernels()
{
  cl_int   status;
  cl_event events[2];

  // clear output array
  cl_manager_->clear_ray_results();

  cl_mem result_buf = clCreateBuffer(cl_manager_->context(),
                                     CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                                     cl_manager_->n_rays() * sizeof(cl_float4),
                                     cl_manager_->ray_results(),
                                     &status);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clCreateBuffer failed. (ray_trace_results)"))
    return SDK_FAILURE;
  else if (this->set_tree_args()!=SDK_SUCCESS)
    return SDK_FAILURE;
  else if (this->set_ray_trace_args()!=SDK_SUCCESS)
    return SDK_FAILURE;

  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          4,
                          sizeof(cl_mem),
                          (void *)&result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_LOCAL_MEM_SIZE,
                                    sizeof(cl_ulong),
                                    &used_local_memory_,
                                    NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_LOCAL_MEM_SIZE failed."))
  {
    return SDK_FAILURE;
  }

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_WORK_GROUP_SIZE,
                                    sizeof(cl_ulong),
                                    &kernel_work_group_size_,
                                    NULL);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_WORK_GROUP_SIZE, failed."))
  {
    return SDK_FAILURE;
  }

  vcl_size_t globalThreads[]= {cl_manager_->n_ray_groups()*
                               cl_manager_->group_size()};
  vcl_size_t localThreads[] = {cl_manager_->group_size()};

  if (used_local_memory_ > cl_manager_->total_local_memory())
  {
    vcl_cout << "Unsupported: Insufficient local memory on device.\n";
    return SDK_FAILURE;
  }
    status = clGetCommandQueueInfo(command_queue_,CL_QUEUE_CONTEXT,0,NULL,0);
  cl_event ceEvent;

  status = clEnqueueNDRangeKernel(command_queue_,
                                  cl_manager_->kernel(),
                                  1,
                                  NULL,
                                  globalThreads,
                                  localThreads,
                                  0,
                                  NULL,
                                  &ceEvent);


  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueNDRangeKernel failed."))
    return SDK_FAILURE;

  status = clFinish(command_queue_);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clFinish failed."))
    return SDK_FAILURE;

  // Enqueue readBuffers
  status = clEnqueueReadBuffer(command_queue_,
                               result_buf,
                               CL_TRUE,
                               0,
                               cl_manager_->n_rays()*sizeof(cl_float4),
                               cl_manager_->ray_results(),
                               0,
                               NULL,
                               &events[0]);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueBuffer (ray_results)failed."))
    return SDK_FAILURE;

  // Wait for the read buffer to finish execution
  status = clWaitForEvents(1, &events[0]);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clWaitForEvents failed."))
    return SDK_FAILURE;

  clReleaseEvent(events[0]);

  status = clReleaseMemObject(result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseMemObject failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::run_ray_creation_test_kernels()
{
  cl_int   status;
  cl_event events[2];

  // clear output array
  cl_manager_->clear_ray_results();

  cl_mem result_buf = clCreateBuffer(cl_manager_->context(),
                                     CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR,
                                     cl_manager_->n_rays()* sizeof(cl_float4),
                                     cl_manager_->ray_results(),
                                     &status);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clCreateBuffer failed. (ray_trace_results)"))
    return SDK_FAILURE;
  if (this->set_camera_args()!=SDK_SUCCESS)
    return SDK_FAILURE;

  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          3,
                          sizeof(cl_mem),
                          (void *)&input_ray_origin_buf_);
  status = clSetKernelArg(cl_manager_->kernel(),
                          4,
                          sizeof(cl_float4),
                          0);
  // the returned array test result
  status = clSetKernelArg(cl_manager_->kernel(),
                          5,
                          sizeof(cl_mem),
                          (void *)&result_buf);
  if (!this->check_val(status,CL_SUCCESS,"clSetKernelArg failed. (result_buf)"))
    return SDK_FAILURE;

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_LOCAL_MEM_SIZE,
                                    sizeof(cl_ulong),
                                    &used_local_memory_,
                                    NULL);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_LOCAL_MEM_SIZE failed."))
  {
    return SDK_FAILURE;
  }

  status = clGetKernelWorkGroupInfo(cl_manager_->kernel(),
                                    cl_manager_->devices()[0],
                                    CL_KERNEL_WORK_GROUP_SIZE,
                                    sizeof(cl_ulong),
                                    &kernel_work_group_size_,
                                    NULL);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clGetKernelWorkGroupInfo CL_KERNEL_WORK_GROUP_SIZE, failed."))
  {
    return SDK_FAILURE;
  }

  vcl_size_t globalThreads[]= {RoundUp(cl_manager_->n_rays(),64)};
  vcl_size_t localThreads[] = {64};

  if (used_local_memory_ > cl_manager_->total_local_memory())
  {
    vcl_cout << "Unsupported: Insufficient local memory on device.\n";
    return SDK_FAILURE;
  }
  cl_event ceEvent;
  status = clEnqueueNDRangeKernel(command_queue_,
                                  cl_manager_->kernel(),
                                  1,
                                  NULL,
                                  globalThreads,
                                  localThreads,
                                  0,
                                  NULL,
                                  &ceEvent);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueNDRangeKernel failed."))
    return SDK_FAILURE;

  status = clFinish(command_queue_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clFinish failed."))
    return SDK_FAILURE;

  // Enqueue readBuffers
  status = clEnqueueReadBuffer(command_queue_,
                               result_buf,
                               CL_TRUE,
                               0,
                               cl_manager_->n_rays()*sizeof(cl_float4),
                               cl_manager_->ray_results(),
                               0,
                               NULL,
                               &events[0]);

  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clEnqueueBuffer (ray_results)failed."))
    return SDK_FAILURE;

  // Wait for the read buffer to finish execution
  status = clWaitForEvents(1, &events[0]);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clWaitForEvents failed."))
    return SDK_FAILURE;

  clReleaseEvent(events[0]);

  status = clReleaseMemObject(result_buf);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseMemObject failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::build_program(bool useimage)
{
  if (cl_manager_->build_kernel_program(useimage)!=CL_SUCCESS)
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::release_kernel()
{
  return cl_manager_->release_kernel();
}

template <class T>
int octree_test_driver<T>::cleanup_tree_test()
{
 cl_int status = SDK_SUCCESS;

 //cl_manager_->cleanup_tree_processing();

 status = clReleaseCommandQueue(command_queue_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseCommandQueue failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
int octree_test_driver<T>::cleanup_ray_test()
{
  cl_int status = SDK_SUCCESS;

  //cl_manager_->cleanup_ray_processing();

  status = clReleaseCommandQueue(command_queue_);
  if (!this->check_val(status,
                       CL_SUCCESS,
                       "clReleaseCommandQueue failed."))
    return SDK_FAILURE;
  else
    return SDK_SUCCESS;
}

template <class T>
octree_test_driver<T>::~octree_test_driver()
{
}

template <class T>
void octree_test_driver<T>::print_kernel_usage_info()
{
  vcl_cout << "Used Local Memory: " << used_local_memory_ << '\n'
           << "Kernel Vetted Work Group Size: " << kernel_work_group_size_ << vcl_endl;
}

#undef OCTREE_TEST_DRIVER_INSTANTIATE
#define OCTREE_TEST_DRIVER_INSTANTIATE(T) \
template class octree_test_driver<T >

#endif // octree_test_driver_txx_
