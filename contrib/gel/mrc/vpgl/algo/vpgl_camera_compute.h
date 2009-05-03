// This is gel/mrc/vpgl/algo/vpgl_camera_compute.h
#ifndef vpgl_camera_compute_h_
#define vpgl_camera_compute_h_
//:
// \file
// \brief Several routines for computing different kinds of cameras from world-point correspondences.
// \author Thomas Pollard
// \date July 18, 2005
//
// Should template this class.

#include <vcl_vector.h>
#include <vgl/vgl_fwd.h>
#include <vnl/vnl_fwd.h>
#include <vpgl/vpgl_calibration_matrix.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vpgl/vpgl_affine_camera.h>
#include <vpgl/vpgl_rational_camera.h>
#include <vgl/algo/vgl_h_matrix_3d.h>

//: Basic least squares solution for a general projective camera given corresponding world and image points.
class vpgl_proj_camera_compute
{
 public:
  //: Compute from two sets of corresponding points.
  // Put the resulting camera into camera, return true if successful.
  static
    bool compute( const vcl_vector< vgl_homg_point_2d<double> >& image_pts,
                  const vcl_vector< vgl_homg_point_3d<double> >& world_pts,
                  vpgl_proj_camera<double>& camera );
  static bool compute( const vcl_vector< vgl_point_2d<double> >& image_pts,
                       const vcl_vector< vgl_point_3d<double> >& world_pts,
                       vpgl_proj_camera<double>& camera );

  //:Find a projective camera that best approximates the rational camera at the world center (lon (deg), lat (deg), elev (meters) )
  static  bool compute(vpgl_rational_camera<double> const& rat_cam,
                       vgl_point_3d<double> const& world_center,
                       vpgl_proj_camera<double>& camera);


  //:An auxiliary matrix that transforms (normalizes) world points prior to projection by a projective camera.  (lon, lat, elevation)->[-1, 1].
 static vgl_h_matrix_3d<double>
   norm_trans(vpgl_rational_camera<double> const& rat_cam);

 private:
  //:default constructor (is private)
  vpgl_proj_camera_compute();
};


//: Basic least squares solution for an affine camera given corresponding world and image points.
class vpgl_affine_camera_compute
{
 public:
  //: Compute from two sets of corresponding points.
  // Put the resulting camera into camera, return true if successful.
 static bool compute( const vcl_vector< vgl_point_2d<double> >& image_pts,
                      const vcl_vector< vgl_point_3d<double> >& world_pts,
                      vpgl_affine_camera<double>& camera );
 private:
  vpgl_affine_camera_compute();
};


//:Various methods for computing a perspective camera
class vpgl_perspective_camera_compute
{
 public:
  //: Compute from two sets of corresponding points.
  // Put the resulting camera into camera, return true if successful.
  static bool compute( const vcl_vector< vgl_point_2d<double> >& image_pts,
                       const vcl_vector< vgl_point_3d<double> >& world_pts,
                       const vpgl_calibration_matrix<double>& K,
                       vpgl_perspective_camera<double>& camera );

  //: Compute from two sets of corresponding 2D points (image and ground plane).
  // \param ground_pts are 2D points representing world points with Z=0
  // The calibration matrix of \a camera is enforced
  // This computation is simplier than the general case above and only requires 4 points
  // Put the resulting camera into camera, return true if successful.
  static bool compute( const vcl_vector< vgl_point_2d<double> >& image_pts,
                       const vcl_vector< vgl_point_2d<double> >& ground_pts,
                       vpgl_perspective_camera<double>& camera );

  //: Compute from a rational camera
  // Put the resulting camera into camera, return true if successful.
  // The approximation volume defines the region of space (lon (deg), lat (deg), elev (meters))
  //  where the perspective approximation is valid. Norm trans is a pre-multiplication
  // of the perspective camera to account for scaling the lon, lat and elevation
  // to the range [-1, 1]
  static bool compute( vpgl_rational_camera<double> const& rat_cam,
                       vgl_box_3d<double> const& approximation_volume,
                       vpgl_perspective_camera<double>& camera,
                       vgl_h_matrix_3d<double>& norm_trans);

  //: Compute from rational camera using a local Euclidean coordinate system.
  static bool compute_local( vpgl_rational_camera<double> const& rat_cam,
                             vgl_box_3d<double> const& approximation_volume,
                             vpgl_perspective_camera<double>& camera,
                             vgl_h_matrix_3d<double>& norm_trans);

 private:
  vpgl_perspective_camera_compute();
};

#endif // vpgl_camera_compute_h_
