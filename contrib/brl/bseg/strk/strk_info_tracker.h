// This is brl/bseg/strk/strk_info_tracker.h
#ifndef strk_info_tracker_h_
#define strk_info_tracker_h_
//---------------------------------------------------------------------
//:
// \file
// \brief a processor for tracking a face_2d based on intensity matching
//
//  The info_tracker operates by randomly generating a set of hypotheses in the
//  vicinity of the previous best n matches. These new hypotheses are tested,
//  (for now by normalized cross-correlation) and ranked to select the best
//  matches for the next iteration.  The current algorithm assumes an
//  equiform transform between frames.
//
// \author
//  J.L. Mundy - August 20, 2003
//
// \verbatim
//  Modifications
//   Restructured to use tracking face - Oct 30, 2003
//   Add Parzen window smoothing - Jan 15, 2004
//   Add Background model - Mar 21, 2004
//   Second Background Attempt - Sept, 2004
//   Add Histogram Feature Capture  - Nov, 2004
// \endverbatim
//
//-------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_memory_image_of.h>
#include <vtol/vtol_face_2d_sptr.h>
#include <vtol/vtol_topology_object_sptr.h>
#include <strk/strk_tracking_face_2d_sptr.h>
#include <strk/strk_info_tracker_params.h>

class strk_info_tracker : public strk_info_tracker_params
{
 public:
  //Constructors/destructor
  strk_info_tracker(strk_info_tracker_params& tp);

  ~strk_info_tracker();
  //Accessors
  void set_image_0(vil1_image& image);
  void set_image_i(vil1_image& image);
  void set_initial_model(vtol_face_2d_sptr const& face);
  void set_capture_face(vtol_face_2d_sptr const& face, bool first_frame=false);
#if 0
  void set_background(vtol_face_2d_sptr const& face);
  vtol_face_2d_sptr current_background();
#endif // 0
  vtol_face_2d_sptr get_best_sample();
  void get_samples(vcl_vector<vtol_face_2d_sptr> & samples);
  strk_tracking_face_2d_sptr tf(int i){return current_samples_[i];}
  void get_best_face_points(vcl_vector<vtol_topology_object_sptr>& points);
  bool get_background_faces(vcl_vector<vtol_face_2d_sptr>& faces);
  strk_tracking_face_2d_sptr initial_tf(){return initial_tf_;}
  strk_tracking_face_2d_sptr capture_tf(){return capture_tf_;}
  vcl_vector<float> histograms();
  vcl_vector<float> capture_histograms(bool first_frame = false);
  //Utility Methods
  bool init();
  bool construct_background_faces(vtol_face_2d_sptr const& current_model,
                                  bool first_frame = false);
  void generate_samples();
  void cull_samples();
  void track();
  void clear();
  //: Evalutate the information at the initial region
  void evaluate_info();
 protected:
  //:random choice to refresh the intensity data of a sample
  bool refresh_sample();
  //: Generate a new tracking face
  strk_tracking_face_2d_sptr
  generate_randomly_positioned_sample(strk_tracking_face_2d_sptr const& seed);
  //: Generate a new tracking face with refreshed data
  strk_tracking_face_2d_sptr
  clone_and_refresh_data(strk_tracking_face_2d_sptr const& sample);
  //: Extract the marginal histograms from the current tracking face
  vcl_vector<float> 
    extract_histograms(strk_tracking_face_2d_sptr const& tf,
                       bool first_frame=false);
#if 0
  //: Construct a multiply connected background face
  bool construct_background_face(vtol_face_2d_sptr& face);
  void refine_best_sample();
#endif // 0
  //members
  vil1_memory_image_of<float> image_0_;  //frame 0 intensity
  vil1_memory_image_of<float> image_i_;  //frame i intensity
  vil1_memory_image_of<float> hue_0_;  //hue of image_0
  vil1_memory_image_of<float> sat_0_;  //saturation of image_0
  vil1_memory_image_of<float> hue_i_;  //hue of image i
  vil1_memory_image_of<float> sat_i_;  //saturation of image_i
  vil1_memory_image_of<float> Ix_0_;  //x derivative of image_0 intensity
  vil1_memory_image_of<float> Iy_0_;  //y derivative of image_0 intensity
  vil1_memory_image_of<float> Ix_i_;  //x derivative of image_i intensity
  vil1_memory_image_of<float> Iy_i_;  //y derivative of image_i intensity
  vtol_face_2d_sptr initial_model_;   //initial model region
  vtol_face_2d_sptr capture_face_;  //pre-stored track face for feature capture
  strk_tracking_face_2d_sptr initial_tf_; //initial tracking face
  strk_tracking_face_2d_sptr capture_tf_; //feature capture tracking face
#if 0
  vtol_face_2d_sptr orig_background_face_; //!< includes model pixels
  vtol_face_2d_sptr background_face_;//!< multiply connected
#endif // 0
  vcl_vector<strk_tracking_face_2d_sptr> current_samples_;
  vcl_vector<strk_tracking_face_2d_sptr> hypothesized_samples_;
  vcl_vector<strk_tracking_face_2d_sptr> track_history_;
  vcl_vector<strk_tracking_face_2d_sptr> background_faces_;
};

#endif // strk_info_tracker_h_
