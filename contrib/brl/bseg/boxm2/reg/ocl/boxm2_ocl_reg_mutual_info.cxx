#include "boxm2_ocl_reg_mutual_info.h"
//:
// \file
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <boxm2/boxm2_block_metadata.h>
#include <boxm2/boxm2_block.h>
#include <boxm2/boxm2_data_base.h>
#include <boxm2/boxm2_util.h>
#include <boxm2/ocl/boxm2_ocl_util.h>
#include <boct/boct_bit_tree.h>
#include <brip/brip_mutual_info.h>
#include <bocl/bocl_mem.h>
#include <bocl/bocl_kernel.h>
#include <vcl_where_root_dir.h>
#include <vcl_algorithm.h>

typedef vnl_vector_fixed<unsigned char,16> uchar16;
bocl_kernel * compile_kernel(bocl_device_sptr device)
{
  vcl_vector<vcl_string> src_paths;
  vcl_string source_dir = vcl_string(VCL_SOURCE_ROOT_DIR) + "/contrib/brl/bseg/boxm2/ocl/cl/";
  vcl_string reg_source_dir = vcl_string(VCL_SOURCE_ROOT_DIR)+ "/contrib/brl/bseg/boxm2/reg/ocl/cl/";
  src_paths.push_back(source_dir     + "scene_info.cl");
  src_paths.push_back(source_dir     + "bit/bit_tree_library_functions.cl");
  src_paths.push_back(reg_source_dir + "estimate_mi_vol.cl");

  bocl_kernel* kern = new bocl_kernel();
  kern->create_kernel(&device->context(),device->device_id(), src_paths, "estimate_mi_vol", "", "MI");

  return kern;
}

//: compute mutual information for a given transformation
bool boxm2_ocl_register_world(boxm2_opencl_cache_sptr& cacheA,
                              boxm2_stream_scene_cache& cacheB,
                              vgl_rotation_3d<double>  rot,
                              vgl_vector_3d<double> tx,
                              int nbins,
                              bocl_device_sptr device,
                              float & mi)
{
  vcl_size_t local_threads[1]={64};
  vcl_size_t global_threads[1]={1};

  boxm2_scene_sptr sceneA = cacheA->get_cpu_cache()->get_scene();
  boxm2_scene_sptr sceneB = cacheB.scene();
  // Instantiate OPENCL
  int status=0;
  cl_command_queue queue = clCreateCommandQueue(device->context(),
                                                *(device->device_id()),
                                                CL_QUEUE_PROFILING_ENABLE,
                                                &status);
  // get scene B on the GPU's host memory
  unsigned int * blk_offsets_array = new unsigned int[cacheB.blk_offsets_.size()];
  for (unsigned k = 0; k< cacheB.blk_offsets_.size(); k++)
    blk_offsets_array[k] = cacheB.blk_offsets_[k];

  unsigned int * alpha_offsets_array = new unsigned int[cacheB.offsets_["alpha"].size()];
  for (unsigned k = 0; k< cacheB.offsets_["alpha"].size(); k++)
      alpha_offsets_array[k] =cacheB.offsets_["alpha"][k]/4;
  //vcl_copy(cacheB.offsets_["alpha"].begin(), cacheB.offsets_["alpha"].end(), alpha_offsets_array);

  vcl_vector<boxm2_block_id> sceneB_ids = sceneB->get_block_ids();
  boxm2_scene_info * sceneB_info = sceneB->get_blk_metadata( sceneB_ids[0] );

  int bbox_buff[9];
  vgl_box_3d<int> bbox = sceneB->bounding_box_blk_ids();

  bbox_buff[0] = bbox.min_x();        bbox_buff[3] = bbox.max_x();
  bbox_buff[1] = bbox.min_y();        bbox_buff[4] = bbox.max_y();
  bbox_buff[2] = bbox.min_z();        bbox_buff[5] = bbox.max_z();

  bbox_buff[6] = sceneB_info->scene_dims[0];
  bbox_buff[7] = sceneB_info->scene_dims[1];
  bbox_buff[8] = sceneB_info->scene_dims[2];

  bocl_mem_sptr sceneB_bbox_ids = new bocl_mem(device->context(), bbox_buff, 9*sizeof(int), " scene B bbox" );
  sceneB_bbox_ids->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  float sceneB_origin_buff[4];
  vgl_box_3d<double> scene_bbox = sceneB->bounding_box();
  sceneB_origin_buff[0] = scene_bbox.min_x();
  sceneB_origin_buff[1] = scene_bbox.min_y();
  sceneB_origin_buff[2] = scene_bbox.min_z();

  bocl_mem_sptr sceneB_origin = new bocl_mem(device->context(), sceneB_origin_buff, 4*sizeof(float), " scene B bbox" );
  sceneB_origin->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  float block_dims[4];
  block_dims[0] = sceneB_info->scene_dims[0]*sceneB_info->block_len;
  block_dims[1] = sceneB_info->scene_dims[1]*sceneB_info->block_len;
  block_dims[2] = sceneB_info->scene_dims[2]*sceneB_info->block_len;
  bocl_mem_sptr sceneB_block_dims = new bocl_mem(device->context(), block_dims, 4*sizeof(float), " scene B block dims" );
  sceneB_block_dims->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  int subblk_num_buff[4];
  subblk_num_buff[0] = sceneB_info->scene_dims[0];
  subblk_num_buff[1] = sceneB_info->scene_dims[1];
  subblk_num_buff[2] = sceneB_info->scene_dims[2];
  subblk_num_buff[3] = sceneB_info->scene_dims[3];

  bocl_mem_sptr sceneB_sub_block_num = new bocl_mem(device->context(), subblk_num_buff, sizeof(int)* 4, " scene B sub block num" );
  sceneB_sub_block_num->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  bocl_mem_sptr sceneB_sub_block_len = new bocl_mem(device->context(), &(sceneB_info->block_len), sizeof(float), " scene B sub block len" );
  sceneB_sub_block_len->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  bocl_mem_sptr ocl_nbins = new bocl_mem(device->context(), &(nbins), sizeof(int), " scene B sub block len" );
  ocl_nbins->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  bocl_mem_sptr blks_ocl_B = new bocl_mem(device->context(), cacheB.blk_buffer_, cacheB.total_bytes_per_block_, " block buffer B" );
  blks_ocl_B->create_buffer(CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR );

  bocl_mem_sptr blks_ocl_B_offsets = new bocl_mem(device->context(), blk_offsets_array, sizeof(unsigned int)*cacheB.blk_offsets_.size(), " block buffer B" );
  blks_ocl_B_offsets->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  bocl_mem_sptr alpha_ocl_B = new bocl_mem(device->context(), cacheB.data_buffers_["alpha"], cacheB.total_bytes_per_data_["alpha"], " alpha buffer B " );
  alpha_ocl_B->create_buffer(CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR );

  bocl_mem_sptr alpha_ocl_B_offsets = new bocl_mem(device->context(), alpha_offsets_array, sizeof(unsigned int)*cacheB.offsets_["alpha"].size(), " block buffer B" );
  alpha_ocl_B_offsets->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  int * joint_histogram_buff= new int[nbins*nbins];
  for (unsigned k = 0 ; k<nbins*nbins; k++)
      joint_histogram_buff[k] = 0;
  bocl_mem_sptr joint_histogram = new bocl_mem(device->context(), joint_histogram_buff, sizeof(int)*nbins*nbins, " joint histogram" );
  joint_histogram->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  float translation_buff[4];
  translation_buff[0] = tx.x();        translation_buff[2] = tx.z();
  translation_buff[1] = tx.y();        translation_buff[3] = 0.0;
#if 0
  vcl_cout<<"Translation  "<<translation_buff[0]<<' '<<translation_buff[1]<<' '<<translation_buff[2]<<vcl_endl;
#endif

  bocl_mem_sptr translation = new bocl_mem(device->context(), translation_buff, sizeof(float)*4, " translation " );
  translation->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  float rotation_buff[9];
  vnl_matrix_fixed<double, 3, 3> R = rot.as_matrix();

  rotation_buff[0] = R(0,0);rotation_buff[3] = R(1,0);rotation_buff[6] = R(2,0);
  rotation_buff[1] = R(0,1);rotation_buff[4] = R(1,1);rotation_buff[7] = R(2,1);
  rotation_buff[2] = R(0,2);rotation_buff[5] = R(1,2);rotation_buff[8] = R(2,2);
#if 0
  vcl_cout<<"Rotation  "<<rot.as_rodrigues()[0]<<' '<<rot.as_rodrigues()[1]<<' '<<rot.as_rodrigues()[2]<<vcl_endl;
#endif
  bocl_mem_sptr rotation = new bocl_mem(device->context(), rotation_buff, sizeof(float)*9, " rotation " );
  rotation->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  bocl_mem_sptr centerX = new bocl_mem(device->context(), boct_bit_tree::centerX, sizeof(cl_float)*585, "centersX lookup buffer");
  bocl_mem_sptr centerY = new bocl_mem(device->context(), boct_bit_tree::centerY, sizeof(cl_float)*585, "centersY lookup buffer");
  bocl_mem_sptr centerZ = new bocl_mem(device->context(), boct_bit_tree::centerZ, sizeof(cl_float)*585, "centersZ lookup buffer");
  centerX->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);
  centerY->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);
  centerZ->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);

  // output buffer for debugging
  float output_buff[1000];
  bocl_mem * output = new bocl_mem(device->context(), output_buff, sizeof(float)*1000, "output" );
  output->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR );

  // bit lookup buffer
  cl_uchar lookup_arr[256];
  boxm2_ocl_util::set_bit_lookup(lookup_arr);
  bocl_mem_sptr lookup=new bocl_mem(device->context(), lookup_arr, sizeof(cl_uchar)*256, "bit lookup buffer");
  lookup->create_buffer(CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);

  // iterate over scene A on the GPU.
  vcl_map<boxm2_block_id, boxm2_block_metadata>& blocks_A = sceneA->blocks();
  vcl_map<boxm2_block_id, boxm2_block_metadata>::iterator blk_iter_A;
  bocl_kernel * kern = compile_kernel(device);

  float gpu_time = 0.0;
  for (blk_iter_A= blocks_A.begin(); blk_iter_A!=blocks_A.end(); blk_iter_A++)
  {
    boxm2_block_metadata mdata = sceneA->get_block_metadata(blk_iter_A->first);
    //write the image values to the buffer
    bocl_mem* blk       = cacheA->get_block(blk_iter_A->first);
    bocl_mem* blk_info  = cacheA->loaded_block_info();
    bocl_mem* alpha     = cacheA->get_data<BOXM2_ALPHA>(blk_iter_A->first,0,false);
    boxm2_scene_info* info_buffer = (boxm2_scene_info*) blk_info->cpu_buffer();
    int alphaTypeSize = (int)boxm2_data_info::datasize(boxm2_data_traits<BOXM2_ALPHA>::prefix());
    info_buffer->data_buffer_length = (int) (alpha->num_bytes()/alphaTypeSize);
    blk_info->write_to_buffer((queue));
    global_threads[0] = (unsigned) RoundUp(mdata.sub_block_num_.x()*mdata.sub_block_num_.y()*mdata.sub_block_num_.z(),(int)local_threads[0]);

    // Kernel ( blk_info , trees, alpha,  transformation, sceneB_bbox,sceneB_bbox_id, blkoffsets, alphaoffsets )
    kern->set_arg(blk_info);
    kern->set_arg(centerX.ptr());
    kern->set_arg(centerY.ptr());
    kern->set_arg(centerZ.ptr());
    kern->set_arg(lookup.ptr());
    kern->set_arg(blk);
    kern->set_arg(alpha);
    kern->set_arg(sceneB_origin.ptr());
    kern->set_arg(sceneB_bbox_ids.ptr());
    kern->set_arg(sceneB_block_dims.ptr());
    kern->set_arg(sceneB_sub_block_len.ptr());
    kern->set_arg(blks_ocl_B.ptr());
    kern->set_arg(alpha_ocl_B.ptr());
    kern->set_arg(blks_ocl_B_offsets.ptr());
    kern->set_arg(alpha_ocl_B_offsets.ptr());
    kern->set_arg(translation.ptr());
    kern->set_arg(rotation.ptr());
    kern->set_arg(ocl_nbins.ptr());
    kern->set_arg(joint_histogram.ptr());
    kern->set_arg(output);
    kern->set_local_arg(nbins*nbins*sizeof(float));
    kern->set_local_arg(16*local_threads[0]*sizeof(unsigned char)); // local trees
    kern->set_local_arg( local_threads[0]*10*sizeof(cl_uchar) );    // cumsum buffer,
    kern->set_local_arg(16*local_threads[0]*sizeof(unsigned char)); // local trees

    kern->execute(queue, 1, local_threads, global_threads);
    int status = clFinish(queue);
    check_val(status, MEM_FAILURE, "UPDATE EXECUTE FAILED: " + error_to_string(status));
    gpu_time += kern->exec_time();
    //clear render kernel args so it can reset em on next execution
    kern->clear_args();

    output->read_to_buffer(queue);
  }
  joint_histogram->read_to_buffer(queue);

  float * joint_histogram_float = reinterpret_cast<float * > (joint_histogram_buff);
  float * histA = new float[nbins];
  float * histB = new float[nbins];
  for (unsigned k = 0 ;k<nbins; k++)
  {
    histA[k] = 0.0;
    histB[k] = 0.0;
  }
  float sum  = 0.0;
  // normalize joint histogram
  for (unsigned k = 0; k < nbins; k++) {
    for (unsigned l = 0; l < nbins; l++) {
      sum+=joint_histogram_float[k*nbins+l];
    }
  }
  for (unsigned k = 0; k < nbins; k++) {
    for (unsigned l = 0; l < nbins; l++) {
      joint_histogram_float[k*nbins+l] = joint_histogram_float[k*nbins+l] / sum;
    }
  }
  for (unsigned k = 0; k < nbins; k++) {
    for (unsigned l = 0; l < nbins; l++) {
       histA[k]+=joint_histogram_float[k*nbins+l];
    }
  }
  for (unsigned k = 0; k < nbins; k++) {
    for (unsigned l = 0; l < nbins; l++) {
      histB[k]+=joint_histogram_float[l*nbins+k];
    }
  }

  float entropyA = 0;
  for (unsigned k = 0; k < nbins; k++) {
    entropyA += -(histA[k]?histA[k]*vcl_log(histA[k]):0); // if prob=0 this value is defined as 0
  }
  float entropyB = 0;
  for (unsigned l = 0; l < nbins; l++) {
    entropyB += -(histB[l]?histB[l]*vcl_log(histB[l]):0); // if prob=0 this value is defined as 0
  }
  float entropyAB =  0.0; ;
  for (unsigned k = 0; k < nbins; k++) {
    for (unsigned l = 0; l < nbins; l++) {
      entropyAB += -(joint_histogram_float[k*nbins+l]?joint_histogram_float[k*nbins+l]*vcl_log(joint_histogram_float[k*nbins+l]):0);
    }
  }
#ifdef DEBUG
  vcl_cout<<"entropyA "<<entropyA<<'\n'
          <<"entropyB "<<entropyB<<'\n'
          <<"entropyAB "<<entropyAB<<vcl_endl;
#endif
  mi = (entropyA +entropyB - entropyAB)/vnl_math::ln2;
  //vcl_cout<<"GPU time is  "<<gpu_time<<vcl_endl;

  // Clean up
  clReleaseCommandQueue(queue);
  delete kern;

  return true;
}
