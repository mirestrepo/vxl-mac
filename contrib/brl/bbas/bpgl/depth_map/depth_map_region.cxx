#include "depth_map_region.h"
//:
// \file
#include <vgl/vgl_plane_3d.h>
#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vgl/algo/vgl_h_matrix_2d_compute_linear.h>
#include <vgl/vgl_intersection.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_polygon_scan_iterator.h>
#include <vgl/vgl_tolerance.h>
#include <vgl/io/vgl_io_plane_3d.h>
#include <vbl/io/vbl_io_smart_ptr.h>
#include <vgl/vgl_closest_point.h>
#include <vsol/vsol_polygon_3d.h>
#include <vsol/vsol_polygon_2d.h>
#include <vsol/vsol_point_2d.h>
#include <vsol/vsol_point_3d.h>
#include <vcl_cassert.h>
#include <vsl/vsl_basic_xml_element.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vnl/algo/vnl_svd.h>
#include <vpgl/algo/vpgl_ray.h>
#include <bpgl/bpgl_camera_utils.h>

// specified and fixed plane
vsol_polygon_3d_sptr depth_map_region::
region_2d_to_3d(vsol_polygon_2d_sptr const& region_2d,
                vgl_plane_3d<double> const& region_plane,
                vpgl_perspective_camera<double> const& cam)
{
  unsigned nverts = region_2d->size();
  vcl_vector<vsol_point_3d_sptr> verts_3d;
  for (unsigned i = 0; i<nverts; ++i) {
    vgl_point_2d<double> pimg = region_2d->vertex(i)->get_p();
    vgl_ray_3d<double> ray = cam.backproject_ray(pimg);
    vgl_point_3d<double> p3d;
    bool success =  vgl_intersection(ray, region_plane, p3d);
    assert(success);
    verts_3d.push_back(new vsol_point_3d(p3d));
  }
  vsol_polygon_3d_sptr poly_3d = new vsol_polygon_3d(verts_3d);
  return poly_3d;
}

vsol_polygon_3d_sptr depth_map_region::
region_2d_to_3d(vsol_polygon_2d_sptr const& region_2d,
                vgl_vector_3d<double> const& region_normal,
                double depth,
                vpgl_perspective_camera<double> const& cam)
{
  if (depth == vcl_numeric_limits<double>::max())
    return 0;
  unsigned nverts = region_2d->size();
  vgl_point_2d<double> centroid = (region_2d->centroid())->get_p();
  // ray from camera through centroid
  vgl_ray_3d<double> cray = cam.backproject_ray(centroid);
  // check if ray is orthogonal to the region normal, i.e.,
  // can't intersect the region plane with the back-projected centroid ray
  vgl_vector_3d<double> dir = cray.direction();
  double dot = dot_product(region_normal, dir);
  bool ortho = vcl_fabs(dot)<= vgl_tolerance<double>::position;
  assert(!ortho);
  vgl_vector_3d<double> vorg(cray.origin().x(), cray.origin().y(), 0.0);
  vgl_vector_3d<double> vxy(dir.x(), dir.y(), 0.0);
  //depth is along centroid direction in the X-Y plane
  double mag = vxy.length();
  assert(mag>0.0);
  double k = depth/mag;
  //but move along camera ray
  vgl_vector_3d<double> p_on_ray = vorg + k*vxy;
  double d = -dot_product(p_on_ray, region_normal);
  vgl_plane_3d<double> plane(region_normal.x(),
                             region_normal.y(),
                             region_normal.z(),
                             d);
  vcl_vector<vsol_point_3d_sptr> verts_3d;
  for (unsigned i = 0; i<nverts; ++i) {
    vgl_point_2d<double> pimg = region_2d->vertex(i)->get_p();
    vgl_ray_3d<double> ray = cam.backproject_ray(pimg);
    vgl_point_3d<double> p3d;
    bool success =  vgl_intersection(ray, plane, p3d);
    assert(success);
    verts_3d.push_back(new vsol_point_3d(p3d));
  }
  vsol_polygon_3d_sptr poly_3d = new vsol_polygon_3d(verts_3d);
  return poly_3d;
}

depth_map_region::depth_map_region() : active_(true),
                                       order_(0), orient_type_(NOT_DEF), name_(""), depth_(-1.0),
                                       min_depth_(0.0), max_depth_(vcl_numeric_limits<double>::max()),
                                       depth_inc_(1.0), region_2d_(0), region_3d_(0)
{}

depth_map_region::depth_map_region(vsol_polygon_2d_sptr const& region,
                                   vgl_plane_3d<double> const& region_plane,
                                   double min_depth, double max_depth,
                                   vcl_string name,
                                   depth_map_region::orientation orient)
  : active_(true), order_(0), orient_type_(orient), name_(name), depth_(-1.0),
    min_depth_(min_depth), max_depth_(max_depth), depth_inc_(1.0),
    region_plane_(region_plane), region_2d_(region), region_3d_(0)
{
}

depth_map_region::depth_map_region(vsol_polygon_2d_sptr const& region,
                                   vgl_plane_3d<double> const& region_plane,
                                   vcl_string name,
                                   depth_map_region::orientation orient)
  : active_(true), order_(0), orient_type_(orient), name_(name), depth_(-1.0),
    min_depth_(-1.0), max_depth_(-1.0),depth_inc_(1.0),
    region_plane_(region_plane), region_2d_(region), region_3d_(0)
{
}

depth_map_region::depth_map_region(vsol_polygon_2d_sptr const& region,
                                   vcl_string name)
  : active_(true), order_(0), orient_type_(INFINT), name_(name),
    depth_(vcl_numeric_limits<double>::max()),
    min_depth_(vcl_numeric_limits<double>::max()),
    max_depth_(vcl_numeric_limits<double>::max()),
    depth_inc_(1.0),
    region_2d_(region), region_3d_(0)
{
}

vsol_point_2d_sptr depth_map_region::centroid_2d() const
{
  if (region_2d_) return region_2d_->centroid();
  return 0;
}

vgl_vector_3d<double> depth_map_region::
perp_ortho_dir(vpgl_perspective_camera<double> const& cam)
{
  // get the principal ray direction
  vgl_vector_3d<double> principal_dir = cam.principal_axis();
  vgl_vector_3d<double> z_dir(0.0, 0.0, 1.0);
  // perpendicular to the z-principal_dir plane
  vgl_vector_3d<double> orth = cross_product(principal_dir, z_dir);
  // perpendicular to orth and perpendicular to the z axis
  vgl_vector_3d<double> orth_perp = cross_product(orth, z_dir);
  return orth_perp;
}

void depth_map_region::
set_region_3d(vpgl_perspective_camera<double> const& cam)
{
  // don't have region plane but is perpendicular to the ground plane
  // and perpendicular to the plane enclosing the camera principal ray and
  // the world z axis, sets region_plane_.
  if (orient_type_ == INFINT) {
    // the plane normal
    vgl_vector_3d<double> normal_dir = depth_map_region::perp_ortho_dir(cam);
    vgl_vector_3d<double> ptv = min_depth_*normalized(cam.principal_axis());
    vgl_point_3d<double> pt(ptv.x(), ptv.y(), ptv.z());
    region_plane_ = vgl_plane_3d<double>(normal_dir, pt);
    this->set_region_3d(min_depth_, cam);
  }
  else
    region_3d_ = depth_map_region::region_2d_to_3d(region_2d_,region_plane_,cam);
}

void depth_map_region::
set_region_3d(double depth, vpgl_perspective_camera<double> const& cam)
{
  vgl_vector_3d<double> normal = region_plane_.normal();
  region_3d_ =
    depth_map_region::region_2d_to_3d(region_2d_, normal, depth, cam);
  depth_ = depth;
  if (region_3d_)
    region_plane_ = vgl_plane_3d<double>(region_3d_->plane());
}

static  vgl_vector_2d<double>
vector_to_closest_horizon_pt(vgl_point_2d<double> p,
                             vgl_line_2d<double> horiz)
{
  vgl_point_2d<double> pcp = vgl_closest_point(p, horiz);
  // get vector along positive z axis
  vgl_vector_2d<double> vp = p-pcp;
  return vp;
}

static vgl_point_2d<double>
move_to_depth(vgl_point_2d<double> const& img_pt, double max_depth,
              vpgl_perspective_camera<double> const& cam,
              vgl_plane_3d<double> const& region_plane)
{
  vgl_ray_3d<double> ray = cam.backproject_ray(img_pt);
  vgl_vector_3d<double> pray = normalized(cam.principal_axis());
  vgl_vector_3d<double> prxy(pray.x(), pray.y(), 0.0);
  double prxy_mag = prxy.length();
  vgl_point_3d<double> p3d, org = ray.origin();
  bool success =  vgl_intersection(ray, region_plane, p3d);
  assert(success);
  //    double depth = vgl_distance(p3d, org);
  // move p3d along (v3dx,v3dy) so that depth = max_depth
  vgl_vector_3d<double> v3d = p3d-org;
  double depth = dot_product(prxy, v3d);
#if 0
  double dd = max_depth*max_depth - depth*depth;
  dd /= (v3d.x()*v3d.x() + v3d.y()*v3d.y());
  double k = -1 + vcl_sqrt(1 + dd);
#endif
  double k = (max_depth - depth)/prxy_mag;
  // project into image
  vgl_vector_3d<double> v3dn = normalized(v3d);
  vgl_vector_3d<double> vmove =
    k*vgl_vector_3d<double>(v3dn.x(), v3dn.y(), 0.0);
  vgl_point_3d<double> pmoved_3d = p3d+vmove;
  vgl_point_2d<double> pmoved_2d(cam.project(pmoved_3d));
  return pmoved_2d;
}

bool depth_map_region::
set_ground_plane_max_depth(double max_depth,
                           vpgl_perspective_camera<double> const& cam,
                           double proximity_scale_factor)
{
  if (orient_type_ != GROUND_PLANE)
    return false;
  double tol = vcl_sqrt(vgl_tolerance<double>::position);
  vgl_line_2d<double> horizon = bpgl_camera_utils::horizon(cam);
  horizon.normalize();
  vgl_point_2d<double> centroid = (region_2d_->centroid())->get_p();
  vgl_point_2d<double> cpcent = vgl_closest_point(centroid, horizon);
  vgl_vector_2d<double> vc = centroid-cpcent;
  double dc = vc.length();

  // first check if the polygon is entirely above the horizion
  // shouldn't happen
  vgl_vector_3d<double> pray = 100.0*normalized(cam.principal_axis());
  vgl_point_3d<double> on_gp(pray.x(), pray.y(), 0.0);
  vgl_point_3d<double> above_gp(pray.x(), pray.y(), 1.0);
  vgl_point_2d<double> p0(cam.project(on_gp));
  vgl_point_2d<double> p1(cam.project(above_gp));
  vgl_vector_2d<double> v01 = p1-p0;
  unsigned nverts = region_2d_->size();
  bool needs_split = false;
  vcl_vector<vsol_point_2d_sptr> pts_above_on;
  vcl_vector<unsigned> pts_closer_than_centroid;
  double min_d = vcl_numeric_limits<double>::max();
  for (unsigned i = 0; i<nverts; ++i) {
    vgl_point_2d<double> p = region_2d_->vertex(i)->get_p();
    vgl_vector_2d<double> vp = vector_to_closest_horizon_pt(p, horizon);
    double d = vp.length();
    if (d<dc)
      pts_closer_than_centroid.push_back(i);
    if (d <min_d) {
      min_d = d;
    }
    double dp = dot_product(vp, v01);
    bool above = dp>0.0;
    bool on = vcl_fabs(dp)<tol;
    if (above || on) {
      pts_above_on.push_back(region_2d_->vertex(i));
      needs_split = true;
    }
  }
  assert(pts_above_on.size()<nverts);
  // assert didn't happen and no points are above the horizon
  if (needs_split) return false;// don't handle this case yet
  double d_thresh = min_d*proximity_scale_factor;
  for (unsigned i = 0; i<pts_closer_than_centroid.size(); ++i) {
    unsigned ip = pts_closer_than_centroid[i];
    vgl_point_2d<double> p = region_2d_->vertex(ip)->get_p();
    vgl_vector_2d<double> vp = vector_to_closest_horizon_pt(p, horizon);
    double d = vp.length();
    if (d < d_thresh)
      {
        vgl_point_2d<double> p_moved = move_to_depth(p, max_depth,cam,region_plane_);
        vsol_point_2d_sptr p_ptr = region_2d_->vertex(ip);
        p_ptr->set_x(p_moved.x()); p_ptr->set_y(p_moved.y());
      }
  }
  this->set_region_3d(cam);
  return true;
}
bool depth_map_region::
img_to_region_plane(vpgl_perspective_camera<double> const& cam,
                    vgl_h_matrix_2d<double>& H) const{
  if(this->orient_type() == INFINT){
    vcl_cout << "homography not defined for regions at infinity\n";
    return false;
  }
  if(!region_3d_){
    vcl_cout << "region 3-d is null - depth not set\n";
    return false;
  }
  vcl_vector<vgl_point_2d<double> > verts_2d, reg_pts_2d;
  unsigned nverts = region_2d_->size();
  for (unsigned i = 0; i<nverts; ++i){
    vgl_point_2d<double> pimg = region_2d_->vertex(i)->get_p();
    verts_2d.push_back(pimg);
    vgl_point_3d<double> p3d = region_3d_->vertex(i)->get_p();
    vgl_point_2d<double> p2d;
    double tol = vcl_sqrt(vgl_tolerance<double>::position);
    bool on_plane = region_plane_.plane_coords(p3d, p2d, tol);
    if(!on_plane){
      vcl_cout << "vertex " << p3d << " not on region plane\n";
      return false;
    }
    reg_pts_2d.push_back(p2d);
  }
  // cast to homogenous points
  vcl_vector<vgl_homg_point_2d<double> > hpts_2d, hpts_reg_2d;
  for (unsigned i = 0; i<nverts; ++i){
    hpts_2d.push_back(vgl_homg_point_2d<double>(verts_2d[i].x(),verts_2d[i].y()) );
    hpts_reg_2d.push_back(vgl_homg_point_2d<double>(reg_pts_2d[i].x(),reg_pts_2d[i].y()));
  }
  vgl_h_matrix_2d_compute_linear hc;
  bool success = hc.compute(hpts_2d, hpts_reg_2d, H);
  return success;
}

bool depth_map_region::
update_depth_image(vil_image_view<float>& depth_image,
                   vpgl_perspective_camera<double> const& cam,
                   double downsample_ratio) const
{
  vgl_vector_3d<double> pray = normalized(cam.principal_axis());
  vgl_point_3d<double> cen = cam.get_camera_center();
  vgl_vector_3d<double> prxy(pray.x(), pray.y(), 0.0);
  vcl_vector<vgl_point_2d<double> > verts_2d, reg_pts_2d;
  unsigned nverts = region_2d_->size();
  for (unsigned i = 0; i<nverts; ++i){
    vgl_point_2d<double> pimg = region_2d_->vertex(i)->get_p();
    verts_2d.push_back(pimg);
  }
  vgl_h_matrix_2d<double> H;
  if (this->orient_type() != INFINT)
    if (!this->img_to_region_plane(cam, H))
      return false;
  vcl_vector<vgl_point_2d<double> > downsmp_verts_2d;
  for (unsigned i = 0; i<nverts; ++i){
    vgl_point_2d<double>& vert = verts_2d[i];
    vgl_point_2d<double> dvert(vert.x()/downsample_ratio,
                               vert.y()/downsample_ratio);
    downsmp_verts_2d.push_back(dvert);
  }
  // now scan the region into the image
  vgl_polygon<double> vpoly(downsmp_verts_2d);
  //scan convert depths
  int ni = depth_image.ni(), nj = depth_image.nj();
  vgl_polygon_scan_iterator<double> scan(vpoly);
  float inf = vcl_numeric_limits<float>::max();
  for (scan.reset(); scan.next(); ) {
    int v = scan.scany();
    for (int u = scan.startx(); u <= scan.endx(); ++u) {
      if (u<0||u>=ni||v<0||v>=nj)
        continue;
      if (this->orient_type() != INFINT){
        double su = u*downsample_ratio, sv = v*downsample_ratio;
        vgl_homg_point_2d<double> hp(su, sv);
        vgl_point_2d<double> hpr;
        hpr = H(hp);
        vgl_point_3d<double> p3d = region_plane_.world_coords(hpr);
        vgl_vector_3d<double> rayv = p3d-cen;
        double depth = rayv.length();
        depth_image(u, v) = static_cast<float>(depth);
      }else
        depth_image(u, v) = inf;
    }
  }
  return true;
}

void vsl_b_write(vsl_b_ostream& os, const depth_map_region* dm_ptr)
{
  if (dm_ptr ==0) {
    vsl_b_write(os, false);
    return;
  }
  else
    vsl_b_write(os, true);
  depth_map_region* dm_non_const = const_cast<depth_map_region*>(dm_ptr);
  dm_non_const->b_write(os);
}

void vsl_b_read(vsl_b_istream &is, depth_map_region*& dm_ptr)
{
  bool valid_ptr = false;
  vsl_b_read(is, valid_ptr);
  if (valid_ptr) {
    dm_ptr = new depth_map_region();
    dm_ptr->b_read(is);
    return;
  }
  dm_ptr = 0;
}

void vsl_b_write(vsl_b_ostream& os, const depth_map_region_sptr& dm_ptr)
{
  depth_map_region* dm=dm_ptr.ptr();
  vsl_b_write(os, dm);
}

void vsl_b_read(vsl_b_istream &is, depth_map_region_sptr& dm_ptr)
{
  depth_map_region* dm=0;
  vsl_b_read(is, dm);
  dm_ptr = dm;
}

//: binary IO write
void depth_map_region::b_write(vsl_b_ostream& os)
{
  unsigned ver = this->version();
  vsl_b_write(os, ver);
  vsl_b_write(os, active_);
  vsl_b_write(os, order_);
  unsigned temp = static_cast<unsigned>(orient_type_);
  vsl_b_write(os, temp);
  vsl_b_write(os, name_);
  vsl_b_write(os, depth_);
  vsl_b_write(os, min_depth_);
  vsl_b_write(os, max_depth_);
  vsl_b_write(os, depth_inc_);
  vsl_b_write(os, region_plane_);
  vsl_b_write(os, region_2d_.ptr());
  vsl_b_write(os, region_3d_.ptr());
}

//: binary IO read
void depth_map_region::b_read(vsl_b_istream& is)
{
  unsigned ver;
  vsl_b_read(is, ver);
  if (ver ==1) {
    vsl_b_read(is, active_);
    vsl_b_read(is, order_);
    unsigned temp;
    vsl_b_read(is, temp);
    orient_type_ = static_cast<orientation>(temp);
    vsl_b_read(is, name_);
    vsl_b_read(is, depth_);
    vsl_b_read(is, min_depth_);
    vsl_b_read(is, max_depth_);
    vsl_b_read(is, depth_inc_);
    vsl_b_read(is, region_plane_);
    vsol_polygon_2d* r2d=0;
    vsl_b_read(is, r2d);
    region_2d_ = r2d;
    vsol_polygon_3d* r3d=0;
    vsl_b_read(is, r3d);
    region_3d_ = r3d;
  }else{
    vcl_cout << "depth_map_region - unknown binary io version " << ver <<'\n';
    return;
  }
}
