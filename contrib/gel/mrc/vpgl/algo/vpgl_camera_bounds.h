// This is gel/mrc/vpgl/algo/vpgl_camera_bounds.h
#ifndef vpgl_camera_bounds_h_
#define vpgl_camera_bounds_h_
//:
// \file
// \brief Methods for computing various bounds on cameras and scenes
// \author J. L. Mundy
// \date September 18, 2010

#include <vcl_vector.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vpgl/vpgl_affine_camera.h>
#include <vpgl/vpgl_perspective_camera.h>

class vpgl_camera_bounds
{
 public:
  //: the solid angle for a pixel.
  // Applies only to perspective camera. Cone is tangent to pixel. Angles in radians and steradians
  static void pixel_solid_angle(vpgl_perspective_camera<double> const& cam,
                                unsigned u, unsigned v,
                                vgl_ray_3d<double>& cone_axis,
                                double& cone_half_angle,
                                double& solid_angle);


  //: the solid angle (and cone half angle) for the pixel at the 
  // principal point, applies only to a perspective camera 
  // cone is tangent to the pixel.
  //Angles in radians and steradians
  static void pixel_solid_angle(vpgl_perspective_camera<double> const& cam,
                                double& cone_half_angle,
                                double& solid_angle);

  //: the solid angle for an image, applies only to perspective camera
  // the cone axis passes through principal point, i.e. principal ray.
  // the cone is tangent to the square defined by the image diagonal
  // angles in radians and steradians
  static void image_solid_angle(vpgl_perspective_camera<double> const& cam,
                                vgl_ray_3d<double>& cone_axis,
                                double& cone_half_angle,
                                double& solid_angle);


  //: the solid angle for an image, applies only to perspective camera
  // same as above but code axis is not returned.
  // angles in radians and steradians
  static void image_solid_angle(vpgl_perspective_camera<double> const& cam,
                                double& cone_half_angle,
                                double& solid_angle);
  //: the solid angle for a scene bounding box, the cone is tangent 
  // to the 3-d scene box
  // angles in radians and steradians
  static bool box_solid_angle(vpgl_perspective_camera<double> const& cam,
                              vgl_box_3d<double> const& box,
                              vgl_ray_3d<double>& cone_axis,
                              double& cone_half_angle,
                              double& solid_angle);

  //: The angular interval of the rotation about the principal axis that bounds a pixel at the smallest image radius, i.e., 1/2 the smallest image dimension.
  //  The angular wedge is tangent to the pixel
  static double rotation_angle_interval(vpgl_perspective_camera<double> const& cam);

 private:
  //: constructor private - class contains static methods only
  vpgl_camera_bounds();
  //: destructor private - class contains static methods only
  ~vpgl_camera_bounds();
};

//: scan the principal ray over a cone defined by the half apex angle
//  it is assumed the cone axis is the positive z direction.
//  returns a 3-d rotation from the positive z direction to the current
//  iterator rotation state. The the cone is scanned in uniform steps
// defined by the increment. The actual number of samples is returned
// by n_samples. The input is the goal number of samples.
class principal_ray_scan
{
 public:
  principal_ray_scan(double cone_half_angle, unsigned& n_samples);
  ~principal_ray_scan() {}
  //: number of scan states
  unsigned n_states(){return theta_.size();}
  //: reset the scan state
  void reset();
  //:the next scan state. Returns false if done
  bool next();
  //: the camera rotation corresponding to the current state of 
  //  the principal ray. alpha is an additional rotation about 
  // the principal ray -pi<alpha<=pi
  vgl_rotation_3d<double> rot(double alpha = 0.0) {return rot(index_, alpha);}
  //: rotation for a given scan index
  vgl_rotation_3d<double> rot(unsigned i, double alpha = 0.0);
  //: theta - spherical elevation angle for current state
  double theta(){return theta_[index_];}
  //: theta - spherical elevation angle for a given scan index
  double theta(unsigned i){return theta_[i];}
  //: phi - spherical azimuth angle for current state
  double phi(){return phi_[index_];}
  //: phi - spherical azimuth angle for a given scan index
  double phi(unsigned i){return phi_[i];}
  //: point on the unit sphere for current scan state
  vgl_point_3d<double> pt_on_unit_sphere(){return pt_on_unit_sphere(index_);}
  //: point on the unit sphere for a given scan index
  vgl_point_3d<double> pt_on_unit_sphere(unsigned i);

 private:
  principal_ray_scan(){};
  unsigned index_;
  vcl_vector<double> theta_;
  vcl_vector<double> phi_;
};

#endif // vpgl_camera_bounds_h_