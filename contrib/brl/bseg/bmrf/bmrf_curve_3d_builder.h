// This is brl/bseg/bmrf/bmrf_curve_3d_builder.h
#ifndef bmrf_curve_3d_builder_h_
#define bmrf_curve_3d_builder_h_
//:
// \file
// \brief A class to build 3D curves from a bmrf_network
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 3/23/04
//
// \verbatim
//  Modifications
// \endverbatim

#include <bmrf/bmrf_curvel_3d_sptr.h>
#include <bmrf/bmrf_network_sptr.h>
#include <bmrf/bmrf_node_sptr.h>
#include <bmrf/bmrf_arc_sptr.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3x4.h>
#include <vnl/vnl_double_4x4.h>
#include <vgl/vgl_vector_3d.h>
#include <vcl_utility.h>
#include <vcl_set.h>
#include <vcl_list.h>
#include <vcl_vector.h>


//: A 3D curve builder
class bmrf_curve_3d_builder
{
 public:
  typedef vcl_pair<bmrf_arc_sptr, bmrf_curvel_3d_sptr> time_match;
  typedef vcl_vector<time_match> match_vector;

  //: Constructor
  bmrf_curve_3d_builder();
  bmrf_curve_3d_builder(bmrf_network_sptr);
  //: Destructor
  ~bmrf_curve_3d_builder() {}

  //: Set the network
  void set_network(const bmrf_network_sptr& network);

  //: Build The curves
  //  Curves with less than \p min_prj projections are removed
  bool build(int min_prj = 3, int min_len = 10, float sigma = 0.5);

  //: Compute the bounding box aligned with vehicle direction
  bool compute_bounding_box(double inlier_fraction = 0.95);

  //: Return the constructed curves
  vcl_set<vcl_list<bmrf_curvel_3d_sptr> > curves() const;

  //: Return the cameras used in the reconstruction
  vcl_vector<vnl_double_3x4> cameras() const;

  //: Return the 3D direction of motion of the curves
  vgl_vector_3d<double> direction() const;

  //: Return the bounding box transformation
  vnl_double_4x4 bb_xform() const;

 protected:
  //: Initialize the intrinsic camera parameters
  void init_intrinsic();

  //: Initialize the camera matrices
  void init_cameras();

  //: Determine the alpha bounds from the network
  void find_alpha_bounds();
  
  //: Build curvels by linking across time through probable arcs
  vcl_set<bmrf_curvel_3d_sptr>
    build_curvels(vcl_set<bmrf_curvel_3d_sptr>& all_curvels, double alpha) const;

  //: extend all curves to the next alpha 
  vcl_set<bmrf_curvel_3d_sptr> 
    extend_curves( vcl_set<vcl_list<bmrf_curvel_3d_sptr>*>& growing_curves, 
                   double alpha );

  //: Find all arcs where both nodes are valid at \p alpha
  vcl_vector<bmrf_arc_sptr> find_arcs_at(double alpha) const;

  //: Reconstruct the 3d location of a curvel from its projections
  void reconstruct_point(bmrf_curvel_3d_sptr curvel) const;

  //: Simultaneously reconstruct all points in a 3d curve
  void reconstruct_curve(vcl_list<bmrf_curvel_3d_sptr>& curve, float sigma = 0.5) const;

  //: Attempt to interpolate artificial values for missing correspondences
  void interp_gaps(vcl_list<bmrf_curvel_3d_sptr>& curve);

  //: Attempt to fill in missing correspondences
  void fill_gaps(vcl_list<bmrf_curvel_3d_sptr>& curve, double da);

  //: Trim the ends of the curve with few correspondences
  void trim_curve(vcl_list<bmrf_curvel_3d_sptr>& curve, int min_prj);

  //: Trim curvels with large deviation in gamma
  void stat_trim_curve(vcl_list<bmrf_curvel_3d_sptr>& curve, double max_std);

  //: Match the \p curvels to the ends of the \p growing_curves
  void append_curvels(vcl_set<bmrf_curvel_3d_sptr>& curvels,
                      vcl_set<vcl_list<bmrf_curvel_3d_sptr>*>& growing_curves,
                      int min_prj);

  //: Return a measure (0.0 to 1.0) of how well \p new_c matches \p prev_c
  double append_correct(const bmrf_curvel_3d_sptr& new_c, const bmrf_curvel_3d_sptr& prev_c) const;

 protected:
  //: The network
  bmrf_network_sptr network_;

  //: Bounds on the alpha values in the network;
  double min_alpha_;
  double max_alpha_;

  vcl_set<vcl_list<bmrf_curvel_3d_sptr> > curves_;

  //: Camera intrinsic parameters
  vnl_double_3x3 K_;

  //: Vector of cameras
  vcl_vector<vnl_double_3x4> C_;
  
  //: 3D direction unit vector
  vgl_vector_3d<double> direction_;

  //: This transform maps the unit cube into the vehicle aligned bounding box
  vnl_double_4x4 bb_xform_;
};

#endif // bmrf_curve_3d_builder_h_
