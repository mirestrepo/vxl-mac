#ifndef bvpl_octree_processes_h_
#define bvpl_octree_processes_h_

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_macros.h>

DECLARE_FUNC_CONS(bvpl_scene_vector_operator_process);
DECLARE_FUNC_CONS(bvpl_scene_kernel_operator_process);
DECLARE_FUNC_CONS(bvpl_block_kernel_operator_process);
DECLARE_FUNC_CONS(bvpl_save_vrml_process);
DECLARE_FUNC_CONS(bvpl_create_scene_process);
DECLARE_FUNC_CONS(bvpl_plane_propagate_process);
DECLARE_FUNC_CONS(bvpl_nonmax_supp_process);
DECLARE_FUNC_CONS(bvpl_compute_gauss_gradients);
DECLARE_FUNC_CONS(bvpl_scene_histogram_process);
DECLARE_FUNC_CONS(bvpl_block_avg_value_process);


//PCA related
DECLARE_FUNC_CONS(bvpl_discover_pca_features_process);
DECLARE_FUNC_CONS(bvpl_compute_pca_test_error_process);
DECLARE_FUNC_CONS(bvpl_compute_pca_error_scene_process);
DECLARE_FUNC_CONS(bvpl_compute_pca_error_block_process);
DECLARE_FUNC_CONS(bvpl_load_pca_error_scenes_process);
DECLARE_FUNC_CONS(bvpl_add_pca_errors_process);
DECLARE_FUNC_CONS(bvpl_load_pca_info_process);
DECLARE_FUNC_CONS(bvpl_pca_project_process);
DECLARE_FUNC_CONS(bvpl_normalize_pca_training_error_process);
DECLARE_FUNC_CONS(bvpl_pca_global_statistics_process);
DECLARE_FUNC_CONS(bvpl_combine_pairwise_statistics_process);
DECLARE_FUNC_CONS(bvpl_global_pca_process);
DECLARE_FUNC_CONS(bvpl_init_global_pca_process);
DECLARE_FUNC_CONS(bvpl_load_global_pca_125_process);

//Taylor related
DECLARE_FUNC_CONS(bvpl_load_taylor_scenes_process);
DECLARE_FUNC_CONS(bvpl_compute_taylor_error_process);
DECLARE_FUNC_CONS(bvpl_add_taylor_errors_process);
DECLARE_FUNC_CONS(bvpl_init_global_taylor_process);
DECLARE_FUNC_CONS(bvpl_compute_taylor_coefficients_process);
DECLARE_FUNC_CONS(bvpl_explore_coefficient_scene_process);

#endif
