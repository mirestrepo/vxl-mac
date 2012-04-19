#include "boxm2_ocl_register.h"

#include <bprb/bprb_macros.h>
#include <bprb/bprb_batch_process_manager.h>
#include <bprb/bprb_func_process.h>

#include "boxm2_ocl_processes.h"
#include <boxm2/ocl/boxm2_opencl_cache.h>

void boxm2_ocl_register::register_datatype()
{
  REGISTER_DATATYPE( boxm2_opencl_cache_sptr );
}

void boxm2_ocl_register::register_process()
{
  // utilities
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_image_process, "boxm2OclRenderExpectedImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_gl_expected_image_process, "boxm2OclRenderGlExpectedImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_depth_process, "boxm2OclRenderExpectedDepthProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_height_map_process, "boxm2OclRenderExpectedHeightMapProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_z_image_process, "boxm2OclRenderExpectedZImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_process, "boxm2OclUpdateProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_with_alt_process, "boxm2OclUpdateWithAltProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_refine_process, "boxm2OclRefineProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_merge_process, "boxm2OclMergeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_color_process, "boxm2OclRenderExpectedColorProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_color_process, "boxm2OclUpdateColorProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_gl_expected_color_process, "boxm2OclRenderGlExpectedColorProcess"); 
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_change_detection_process, "boxm2OclChangeDetectionProcess"); 
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_visualize_change_process, "boxm2OclVisualizeChangeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_filter_process, "boxm2OclFilterProcess");

  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_aux_per_view_process, "boxm2OclUpdateAuxPerViewProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_create_norm_intensities_process, "boxm2OclCreateNormIntensitiesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_create_aux_process, "boxm2OclCreateAuxProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_aux_update_view_direction_process, "boxm2OclAuxUpdateViewDirectionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_batch_synoptic_function_process, "boxm2OclBatchSynopticFunctionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_alpha_wcubic_process, "boxm2OclUpdateAlphaWcubicProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_uncertainty_per_image_process, "boxm2OclUncertaintyPerImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_batch_uncertainty_process, "boxm2OclBatchUncertaintyProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_scene_uncertainty_map_process, "boxm2OclRenderSceneUncertaintyMapProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_synoptic_update_alpha_process, "boxm2OclSynopticUpdateAlphaProcess");
  
  
  //batch paint
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_paint_batch_process, "boxm2OclPaintBatchProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_paint_online_process, "boxm2OclPaintOnlineProcess");
  
  // research
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_histogram_process, "boxm2OclUpdateHistogramProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_batch_probability_process, "boxm2OclBatchProbabilityProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_probability_of_image_process, "boxm2OclProbabilityOfImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_probability_of_image_gl_process, "boxm2OclProbabilityOfImageGlProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_sun_visibilities_process, "boxm2OclUpdateSunVisibilitiesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_shadow_map_process, "boxm2OclRenderExpectedShadowMapProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_flip_normals_using_vis_process, "boxm2OclFlipNormalsUsingVisProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_aggregate_normal_from_filter_process, "boxm2OclAggregateNormalFromFilterProcess");
  REG_PROCESS_FUNC_CONS2(boxm2_ocl_aggregate_normal_from_filter_vector_process);
  REG_PROCESS_FUNC_CONS2(boxm2_ocl_kernel_filter_process);
  REG_PROCESS_FUNC_CONS2(boxm2_ocl_kernel_vector_filter_process);

  // tools
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_query_hist_data_process, "boxm2OclQueryHistDataProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_query_cell_data_process, "boxm2OclQueryCellDataProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_convert_float_image_to_rgba_process, "boxm2OclConvertFloatImageToRgbaProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_ingest_dem_process, "boxm2OclIngestDemProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_ingest_buckeye_dem_process, "boxm2OclIngestBuckeyeDemProcess");
  
  //alternative ray trace
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_cone_expected_image_process, "boxm2OclRenderConeExpectedImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_cone_update_process, "boxm2OclConeUpdateProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_adaptive_cone_update_process, "boxm2OclAdaptiveConeUpdateProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_adaptive_cone_render_expected_process, "boxm2OclAdaptiveConeRenderExpectedProcess");

  // admin
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_create_opencl_cache_process, "boxm2CreateOpenclCacheProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_clear_opencl_cache_process, "boxm2ClearOpenclCacheProcess");

  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_aux_per_view_naa_process, "boxm2OclUpdateAuxPerViewNAAProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_update_alpha_naa_process, "boxm2OclUpdateAlphaNAAProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_image_naa_process, "boxm2OclRenderExpectedImageNAAProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_render_expected_albedo_normal_process, "boxm2OclRenderExpectedAlbedoNormalProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_ocl_batch_compute_normal_albedo_array_process, "boxm2OclBatchComputeNormalAlbedoArrayProcess");

}
