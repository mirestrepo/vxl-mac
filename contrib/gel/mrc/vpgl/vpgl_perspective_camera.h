#ifndef VPGL_PERSPECTIVE_CAMERA_H
#define VPGL_PERSPECTIVE_CAMERA_H

// Intrinsic Camera Parameters
// ---------------------------
// f = Focal Length (in whatever units of length).
// ku = Number of pixels per unit length along the horizontal u-axis in
//      the image.  Alternatively, 1/ku = size in units of length, of
//      horizontal pixel.
// kv = Number of pixels per unit length along the vertical v-axis in
//      the image.  Alternatively, 1/kv = size in units of length, of
//      vertical pixel.
// uo = u-coordinate of the principal point.
// vo = v-coordinate of the principal point.
// theta = Angle (in radians) between the u & v axes in the image.
//         Typically axes are orthogonal, ie. theta = 90.0.
//
// Extrinsic Camera Parameters
// ---------------------------
// <XL, YL, ZL> = World Coordinates of the Camera Center.
// <omega, phi, kappa> = Rotations about the X, Y and Z axes resp, which
//                       take the world coordinate system to the camera
//                       coordinate system.  These rotations are specified
//                       in Degrees.  Defaults to 0.0 each.
//
// .SECTION Modifications :
//
//-----------------------------------------------------------------------------

#include <vpgl/vpgl_matrix_camera.h>
#include <vpgl/vpgl_matrix_camera_sptr.h>
#include <vcsl/vcsl_matrix.h>
#include <vcsl/vcsl_cartesian_3d.h>
#include <vcsl/vcsl_graph_sptr.h>

class vpgl_perspective_camera : public vpgl_basic_camera
{
public:
  enum PerspParams{f=0, ku, kv, uo, vo, theta, XL, YL, ZL, omega, phi, kappa, parent};

  // Constructors/Initializers/Destructors-------------------------------------
  vpgl_perspective_camera();
  vpgl_perspective_camera(vcl_vector<double> &);
  ~vpgl_perspective_camera();

  // Virtual Casting methods-------------------------
  virtual vpgl_perspective_camera *  get_perspective_camera() { return this;}
  virtual void world_to_image(const vnl_vector<double>& vect3d,
                              double& imgu, double& imgv,double time) ;
  virtual void world_to_image(const double& x, const double& y,
                              const double& z, double& ix, double& iy,double time);

  // Data Access---------------------------------------------------------------
  void print_data(vcl_ostream &strm) const;
  double& operator() (PerspParams param_index);
  vcl_vector<double> get_params();
  void set_params(vcl_vector<double> new_params,const vcsl_spatial_sptr & acs);
  vnl_vector<double> get_position() const;

  void update_intrinsic();
  void update();
  void set_lcs(const vcsl_spatial_sptr & new_lcs);
vcsl_spatial_sptr  get_lcs();
void set_acs(const vcsl_spatial_sptr &new_acs);
vcsl_spatial_sptr  get_acs();
protected:
private:
  // Data Members--------------------------------------------------------------

  // Projection matrix from 3d to 2d
  vpgl_matrix_camera_sptr _mat_cam;
  // Paramters of projection matrix
  vcl_vector<double> _mat_params;

  void update_matrix_camera();

protected:
   //Phsical properties which will be converted to projection matrix
  vcl_vector<double> _params;
   //Local coordinate system of the camera
  vcsl_spatial_sptr lcs;
   //Absolute coordinate system.
  vcsl_spatial_sptr acs;
  vcsl_graph_sptr graph;
};

#endif // _vpgl_perspective_camera_h
