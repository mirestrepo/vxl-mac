// This is contrib/bvxm/rec/bvxm_part_hierarchy_builder.h
#ifndef bvxm_part_hierarchy_builder_h_
#define bvxm_part_hierarchy_builder_h_
//:
// \file
// \brief class to build a hiearachy of composable parts for recognition
//        includes tools to learn co-occurrence statistics or manual construction of hierarchies for specific purposes
//
// \author Ozge C Ozcanli (ozge@lems.brown.edu)
// \date 10/16/08
//      
// \verbatim
//   Modifications
//  
// \endverbatim
//
//

#include <rec/bvxm_part_hierarchy_sptr.h>
#include <rec/bvxm_part_base_sptr.h>
#include <vil/vil_image_resource_sptr.h>

class bvxm_part_hierarchy_builder {
public:
  
  //: uses 4 gaussians as primitives
  static bvxm_part_hierarchy_sptr construct_candidates_from_one_image(vil_image_resource_sptr img, float min_strength);

  //: construct layer_n from all pairwise combinations of detected parts of layer_n-1
  static bool construct_layer_candidates(unsigned layer_n, bvxm_part_hierarchy_sptr& h, vcl_vector<bvxm_part_instance_sptr>& parts);

  //: construct a hierarchy manually
  static bvxm_part_hierarchy_sptr construct_vehicle_detector();
  static bvxm_part_hierarchy_sptr construct_vehicle_detector_roi1_0();
  static bvxm_part_hierarchy_sptr construct_vehicle_detector_roi1_1();
  static bvxm_part_hierarchy_sptr construct_vehicle_detector_roi1_2(); // recognize sides of roads, good to remove artifacts from change map


};

#endif  //bvxm_part_hierarchy_builder_h_
