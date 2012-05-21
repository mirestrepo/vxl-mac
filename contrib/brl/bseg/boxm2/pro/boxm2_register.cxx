#include "boxm2_register.h"

#include <bprb/bprb_macros.h>
#include <bprb/bprb_batch_process_manager.h>
#include <bprb/bprb_func_process.h>

#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>
#include <boxm2/io/boxm2_stream_cache.h>

#include <imesh/imesh_mesh.h>

#include "boxm2_processes.h"

void boxm2_register::register_datatype()
{
  REGISTER_DATATYPE( boxm2_scene_sptr );
  REGISTER_DATATYPE( boxm2_cache_sptr );
  REGISTER_DATATYPE( imesh_mesh_sptr );
  REGISTER_DATATYPE( boxm2_stream_cache_sptr );
}

void boxm2_register::register_process()
{
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_load_scene_process,      "boxm2LoadSceneProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_create_cache_process,    "boxm2CreateCacheProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_create_scene_process,    "boxm2CreateSceneProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_write_scene_xml_process, "boxm2WriteSceneXMLProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_add_block_process,       "boxm2AddBlockProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_export_textured_mesh_process,"boxm2ExportTexturedMeshProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_export_mesh_process,"boxm2ExportMeshProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_texture_mesh_process,"boxm2TextureMeshProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_describe_scene_process,"boxm2DescribeSceneProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_create_stream_cache_process, "boxm2CreateStreamCacheProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_roi_init_process, "boxm2RoiInitProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_scene_illumination_info_process, "boxm2SceneIlluminationInfoProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_persp_cam_from_scene_process, "boxm2PerspCamFromSceneProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_write_cache_process, "boxm2WriteCacheProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_stream_cache_close_files_process, "boxm2StreamCacheCloseFilesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_compute_sun_affine_camera_process, "boxm2ComputeSunAffineCameraProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_mask_sift_features_process, "boxm2MaskSiftFeaturesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_bundle_to_scene_process, "boxm2BundleToSceneProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_clear_cache_process, "boxm2ClearCacheProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_blob_change_detection_process, "boxm2BlobChangeDetectionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_blob_precision_recall_process, "boxm2BlobPrecisionRecallProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_export_oriented_point_cloud_process, "boxm2ExportOrientedPointCloudProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_extract_point_cloud_process, "boxm2ExtractPointCloudProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_scene_bbox_process, "boxm2SceneBboxProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_transform_model_process, "boxm2TransformModelProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_create_scene_mask_process, "boxm2CreateSceneMaskProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_paint_mesh_process, "boxm2PaintMeshProcess");
  REG_PROCESS_FUNC_CONS2(boxm2_add_aux_info_to_ply_process);

}
