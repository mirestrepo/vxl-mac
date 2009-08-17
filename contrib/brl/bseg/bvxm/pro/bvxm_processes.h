#ifndef bvxm_processes_h_
#define bvxm_processes_h_

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_macros.h>

// Main framework
DECLARE_FUNC_CONS(bvxm_render_expected_image_process);
DECLARE_FUNC_CONS(bvxm_render_virtual_view_process);
DECLARE_FUNC_CONS(bvxm_roc_process);
DECLARE_FUNC_CONS(bvxm_roi_init_process);
DECLARE_FUNC_CONS(bvxm_rpc_registration_process);
DECLARE_FUNC_CONS(bvxm_change_detection_display_process);
DECLARE_FUNC_CONS(bvxm_detect_changes_process);
DECLARE_FUNC_CONS(bvxm_clean_world_process);
DECLARE_FUNC_CONS(bvxm_compare_rpc_process);
DECLARE_FUNC_CONS(bvxm_create_local_rpc_process);
DECLARE_FUNC_CONS(bvxm_create_normalized_image_process);
DECLARE_FUNC_CONS(bvxm_create_voxel_world_process);
DECLARE_FUNC_CONS(bvxm_gen_synthetic_world_process);
DECLARE_FUNC_CONS(bvxm_heightmap_process);
DECLARE_FUNC_CONS(bvxm_illum_index_process);
DECLARE_FUNC_CONS(bvxm_normalize_image_process);
DECLARE_FUNC_CONS(bvxm_ocp_compare_process);
DECLARE_FUNC_CONS(bvxm_ocp_hist_process);
DECLARE_FUNC_CONS(bvxm_pmap_hist_process);
DECLARE_FUNC_CONS(bvxm_pmap_ratio_process);
DECLARE_FUNC_CONS(bvxm_update_process);
DECLARE_FUNC_CONS(bvxm_detect_scale_process);
DECLARE_FUNC_CONS(bvxm_compare_3d_voxels_process);
DECLARE_FUNC_CONS(bvxm_create_mog_image_process);
DECLARE_FUNC_CONS(bvxm_locate_region_process);
DECLARE_FUNC_CONS(bvxm_get_grid_process);


//io
DECLARE_FUNC_CONS(bvxm_save_occupancy_raw_process);
DECLARE_FUNC_CONS(bvxm_save_occupancy_vff_process);
DECLARE_FUNC_CONS(bvxm_test_process);

//Edges and lidar
DECLARE_FUNC_CONS(bvxm_update_edges_lidar_process);
DECLARE_FUNC_CONS(bvxm_update_edges_process);
DECLARE_FUNC_CONS(bvxm_update_lidar_process);
DECLARE_FUNC_CONS(bvxm_lidar_edge_detection_process);
DECLARE_FUNC_CONS(bvxm_lidar_init_process);
DECLARE_FUNC_CONS(bvxm_detect_edges_process);
DECLARE_FUNC_CONS(bvxm_create_synth_lidar_data_process);
DECLARE_FUNC_CONS(bvxm_save_edges_raw_process);

//algo
DECLARE_FUNC_CONS(bvxm_merge_mog_process);
DECLARE_FUNC_CONS(bvxm_mog_to_mpm_process);
DECLARE_FUNC_CONS(bvxm_mog_l2_distance);

#endif
