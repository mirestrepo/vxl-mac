#ifndef icam_view_sphere_h_
#define icam_view_sphere_h_
//:
// \file
#include "icam_view_metadata.h"
#include <vsph/vsph_view_sphere.h>
#include <vsph/vsph_view_point.h>
#include <vpgl/vpgl_camera.h>
#include <vil/vil_image_view.h>
#include <vbl/vbl_smart_ptr.h>
#include <vsl/vsl_binary_io.h>

class icam_view_sphere : public vbl_ref_count
{
 public:
  //: Constructor, creates a view sphere around the box, centered at box's center with radius
  icam_view_sphere(): view_sphere_(0),ICAM_LOCAL_MIN_THRESH_(0.0){}
  icam_view_sphere(vgl_box_3d<double> bb, double radius);

  //: Destructor
  ~icam_view_sphere(){ delete view_sphere_; }

  //: creates uniform view points on the view square and generates expected and depth images at each view point
  void create_view_points(double cap_angle, double view_angle, unsigned ni, unsigned nj);

  //: returns the cameras of the view points, associated with the view point id
  void cameras(vcl_map<unsigned, vpgl_camera_double_sptr> &cameras);

  void set_cameras(vcl_map<unsigned, vpgl_camera_double_sptr> const &cameras);

  //: sets the images and depth images, associated with the view point id
  virtual void set_images(vcl_map<unsigned, vil_image_view<float>*>& images,
                          vcl_map<unsigned,vil_image_view<double>*>& depth_images);

  //: computes the camera registration errors for a given image
  virtual void register_image(vil_image_view<float> const& source_img,vpgl_perspective_camera<double>& test_cam);

  //: computes the camera registration errors local minima for a given image
  void find_local_minima(vcl_vector<vsph_view_point<icam_view_metadata> >& local_minima);

  //: given a camera, find the relative camera rotation and translations for each view point
  void camera_transf(vpgl_perspective_camera<double> const& cam);

  inline short version() const { return 1; }

  void b_read(vsl_b_istream &is);

  void b_write(vsl_b_ostream &os) const ;

 protected:
  vsph_view_sphere<vsph_view_point<icam_view_metadata> >* view_sphere_;

  const double ICAM_LOCAL_MIN_THRESH_;
};

typedef vbl_smart_ptr<icam_view_sphere> icam_view_sphere_sptr;

void vsl_b_read(vsl_b_istream const& is, icam_view_sphere &sp);
void vsl_b_write(vsl_b_ostream &os, icam_view_sphere const& sp);
void vsl_b_read(vsl_b_istream const& is, icam_view_sphere_sptr &sp);
void vsl_b_write(vsl_b_ostream &os, icam_view_sphere_sptr const& sp);


#endif
