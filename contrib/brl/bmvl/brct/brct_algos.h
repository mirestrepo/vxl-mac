//:
// \file
// \brief collection class for various reconstruction functions
// \author Kongbin Kang (Kongbin_Kang@Brown.edu)
// \date   4/24/2003
// \verbatim
//  Modifications
//   none yet
// \endverbatim

#ifndef bcrt_algos_h_
#define bcrt_algos_h_

#if defined(_MSC_VER) && ( _MSC_VER > 1000 )
#pragma once
#endif // _MSC_VER > 1000
#include <vcl_fstream.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3x4.h>
#include <vsol/vsol_box_3d_sptr.h>
#include <vsol/vsol_point_3d_sptr.h>
#include <vdgl/vdgl_digital_curve_sptr.h>
#include <bugl/bugl_gaussian_point_2d.h>
#include <bugl/bugl_gaussian_point_3d.h>

struct brct_error_index
{
  brct_error_index(int i, double error){i_ = i; e_ = error;}
  ~brct_error_index(){};
  int i(){return i_;}
  double error(){return e_;}
private:
  int i_;
  double e_;
};


class brct_algos
{
public:
  brct_algos();
  virtual ~brct_algos();

  //operators
public:
  static void add_box_vrml(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);
  static vsol_box_3d_sptr get_bounding_box(vcl_vector<vgl_point_3d<double> > &pts_3d);
  static vgl_point_3d<double> bundle_reconstruct_3d_point(vcl_vector<vnl_double_2> &pts,
                                                          vcl_vector<vnl_double_3x4> &P);
  static vgl_point_2d<double> projection_3d_point(const vgl_point_3d<double> & x, const vnl_double_3x4& P);

  static bugl_gaussian_point_2d<double> project_3d_point(const vnl_double_3x4 &P,
                                                         const bugl_gaussian_point_3d<double> & X);

  static vnl_double_2 projection_3d_point(const vnl_double_3x4& P, const vnl_double_3 & X);
  //: get closet point from a digital curve
  static vgl_point_2d<double>  closest_point(vdgl_digital_curve_sptr dc, vgl_point_2d<double> pt);
  //: get point on a digital curve which is most possible to the guassian pdf
  static vgl_point_2d<double> most_possible_point(vdgl_digital_curve_sptr dc, bugl_gaussian_point_2d<double> &pt);

  //: pointwise reconstruction
  static vgl_point_3d<double> triangulate_3d_point(const vgl_point_2d<double>& x1, const vnl_double_3x4& P1,
                                                   const vgl_point_2d<double>& x2, const vnl_double_3x4& P2);
  //: filter outliers for camera translation
  static void filter_outliers(const vnl_double_3x3& K,
                              const vnl_double_3& trans,
                              vcl_vector<vnl_double_2> & pts_2d,
                              vcl_vector<vnl_double_3> & pts_3d,
                              double fraction = 0.1);

  //: find camera translation from matched 2-d/3-d points
  static bool camera_translation(const vnl_double_3x3& K,
                                 vcl_vector<vnl_double_2> & pts_2d,
                                 vcl_vector<vnl_double_3> & pts_3d,
                                 vnl_double_3& trans);

  //: use uncertainty and point weeding to improve solution
  static void robust_camera_translation(const vnl_double_3x3& K,
                                        vcl_vector<bugl_gaussian_point_2d<double> > & pts_2d,
                                        vcl_vector<vgl_point_3d<double> > & pts_3d,
                                        vnl_double_3& trans);

  //: compute camera translation using epipolar geometry
  static void camera_translation(vnl_double_3x3 const & K,
                                 vnl_double_2 const& image_epipole,
                                 vcl_vector<vnl_double_2> const& points_0,
                                 vcl_vector<vnl_double_2> const& points_1,
                                 vcl_vector<vnl_double_2> const& points,
                                 vnl_double_3& T);

  static double motion_constant(vnl_double_2 const& image_epipole,
                                int i,
                                vnl_double_2& p_i, 
                                vnl_double_2& p_i1); 
                               
  static void print_motion_array(vnl_matrix<double>& H);
  static void read_vrml_points(vcl_ifstream& str, 
                               vcl_vector<vsol_point_3d_sptr>& pts3d);
  
  static void write_vrml_header(vcl_ofstream& str);
  static void write_vrml_trailer(vcl_ofstream& str);
  static void write_vrml_points(vcl_ofstream& str, 
                                vcl_vector<vsol_point_3d_sptr> const& pts3d);
  static void write_vrml_box(vcl_ofstream& str, vsol_box_3d_sptr const& box,
                             const float r = 1.0, const float g = 1.0,
                             const float b = 1.0, 
                             const float transparency = 0.0);
};


#endif // bcrt_algos_h_
