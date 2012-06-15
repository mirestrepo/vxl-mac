#include "vil_register.h"
#include "vil_processes.h"

#include <bprb/bprb_macros.h>
#include <bprb/bprb_func_process.h>
#include <bprb/bprb_batch_process_manager.h>

#include <vil/vil_image_view_base.h>
#include <bil/bil_raw_image_istream.h>

void vil_register::register_datatype()
{
  REGISTER_DATATYPE(vil_image_view_base_sptr);
  REGISTER_DATATYPE(bil_raw_image_istream_sptr); 
}

void vil_register::register_process()
{
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_binary_image_op_process, "vilBinaryImageOpProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_combine_grey_images_process, "vilCombineGreyImagesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_convert_to_n_planes_process, "vilConvertToNPlanesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_crop_image_process, "vilCropImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_gaussian_process, "vilGaussianProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_gradient_process, "vilGradientProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_pair_process, "vilImagePairProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_load_from_ascii_process, "vilLoadFromAsciiProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_load_image_view_binary_process, "vilLoadImageViewBinaryProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_load_image_view_process, "vilLoadImageViewProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_map_image_binary_process, "vilMapImageBinaryProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_map_image_process, "vilMapImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_rgbi_to_grey_process, "vilRGBIToGreyProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_save_image_view_binary_process, "vilSaveImageViewBinaryProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_save_image_view_process, "vilSaveImageViewProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_stretch_image_process, "vilStretchImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_threshold_image_process, "vilThresholdImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_size_process, "vilImageSizeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_two_planes_composite_process, "vilTwoPlanesCompositeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, bil_compass_edge_detector_process, "bilCompassEdgeDetectorProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_resample_process, "vilResampleProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_filter_image_process, "vilImageFilterProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_blob_detection_process, "vilBlobDetectionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_pixelwise_roc_process, "vilPixelwiseRocProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_rgb_to_grey_process, "vilRGBToGreyProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_nitf_date_time_process, "vilNITFDateTimeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_convert_pixel_type_process, "vilConvertPixelTypeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_median_filter_process, "vilMedianFilterProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_gradient_angle_process, "vilGradientAngleProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_ssd_process, "vilImageSSDProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_mean_process, "vilImageMeanProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_range_process, "vilImageRangeProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_init_float_image_process, "vilInitFloatImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, bil_create_raw_image_istream_process, "bilCreateRawImageIstreamProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, bil_read_frame_process, "bilReadFrameProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, bil_seek_frame_process, "bilSeekFrameProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_pixel_value_process, "vilPixelValueProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_scale_and_offset_values_process, "vilScaleAndOffsetValuesProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_set_float_image_pixel_process, "vilSetFloatImagePixelProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_sum_process, "vilImageSumProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_dilate_disk_process, "vilImageDilateDiskProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_shadow_detection_process, "vilShadowDetectionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_shadow_ridge_detection_process, "vilShadowRidgeDetectionProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_mean_and_variance_process, "vilImageMeanAndVarianceProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, vil_image_normalise_process, "vilImageNormaliseProcess");
}

