// This is gel/vdgl/vdgl_interpolator_cubic.h
#ifndef vdgl_interpolator_cubic_h
#define vdgl_interpolator_cubic_h
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief Represents a 2D interpolator_cubic for a vdgl_edgel_chain
// \author
//    Geoff Cross

#include <vdgl/vdgl_interpolator.h>

//: Represents a 2D interpolator_cubic for a vdgl_edgel_chain

class vdgl_interpolator_cubic : public vdgl_interpolator
{
 public:

  // Constructors/Destructors--------------------------------------------------

  vdgl_interpolator_cubic( vdgl_edgel_chain_sptr chain);
  ~vdgl_interpolator_cubic();

  // Operators----------------------------------------------------------------

  //: interpolation 0th degree
  double get_x( const double index);
  double get_y( const double index);

  //: interpolation 1st degree
  double get_theta( const double index);
  double get_tangent_angle( const double index);

  //: interpolation 2nd degree
  double get_curvature( const double index);

  //: integral
  double get_length();

  //: bounding box
  double get_min_x();
  double get_max_x();
  double get_min_y();
  double get_max_y();

  // INTERNALS-----------------------------------------------------------------
 protected:
  // Data Members--------------------------------------------------------------

  double lengthcache_;
  double minxcache_;
  double maxxcache_;
  double minycache_;
  double maxycache_;

 private:
  // Helpers-------------------------------------------------------------------

  void recompute_all();
  void recompute_length();
  void recompute_bbox();
};

#endif // vdgl_interpolator_cubic_h
