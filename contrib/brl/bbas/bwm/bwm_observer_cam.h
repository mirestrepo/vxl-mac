#ifndef bwm_observer_cam_h_
#define bwm_observer_cam_h_
//:
// \file

#include "bwm_observer_vgui.h"
#include "bwm_observable_sptr.h"

#include <vcl_iostream.h>

#include <vgui/vgui_easy2D_tableau.h>
#include <vgui/vgui_viewer2D_tableau.h>

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_box_3d.h>

#include <vsol/vsol_point_2d_sptr.h>
#include <vsol/vsol_point_3d_sptr.h>
#include <vsol/vsol_polygon_2d_sptr.h>
#include <vsol/vsol_polygon_3d_sptr.h>
#include <vnl/vnl_math.h>

#include <vpgl/vpgl_camera.h>
#include <depth_map/depth_map_scene.h>

void bwm_project_meshes(vcl_vector<vcl_string> paths,
                        vpgl_camera<double>* cam,
                        vcl_vector<vgl_polygon<double> > &poly_2d_list);

class bwm_observer_cam : public bwm_observer_vgui
{
 public:

  typedef bwm_observer_vgui base;

  bwm_observer_cam(bgui_image_tableau_sptr const& img, vpgl_camera<double> *camera, vcl_string cam_path)
  : bwm_observer_vgui(img), sun_elev_angle_(vnl_math::pi_over_4), sun_azim_angle_(vnl_math::pi_over_4),
    camera_(camera), cam_path_(cam_path), cam_adjusted_(false),
    proj_plane_(vgl_plane_3d<double>(0, 0, 1, 0)), extrude_mode_(false), show_geo_position_(false), focal_length_(3000.0), cam_height_(1.6),horizon_(0),
    horizon_soview_(0)
  {}
  // set the initial projection plane to z=0
  bwm_observer_cam(bgui_image_tableau_sptr const& img, const char* /*n*/="unnamed")
  : bwm_observer_vgui(img), sun_elev_angle_(vnl_math::pi_over_4), sun_azim_angle_(vnl_math::pi_over_4),
    camera_(0), cam_adjusted_(false),
    proj_plane_(vgl_plane_3d<double>(0, 0, 1, 0)), extrude_mode_(false), show_geo_position_(false), focal_length_(3000.0), cam_height_(1.6),horizon_(0),
    horizon_soview_(0)
  {}

  virtual ~bwm_observer_cam() { delete camera_; }

  bgui_image_tableau_sptr image_tableau() { return img_tab_; }

  vcl_string camera_path() const { return cam_path_; }

  void set_camera_path(vcl_string const& cam_path) { cam_path_=cam_path; }

  bool handle(const vgui_event &e);

  virtual vcl_string type_name() const { return "bwm_observer_cam"; }

  void set_camera(vpgl_camera<double> *camera, vcl_string cam_path)
  { camera_ = camera; cam_path_ = cam_path; cam_adjusted_ = false; }

  vpgl_camera<double> * camera() { return camera_; }

  //: function to project shadow
  void project_shadow();

  bool camera_adjusted() const { return cam_adjusted_; }

  void set_camera_adjusted(bool status) { cam_adjusted_ = status; }

  void set_proj_plane(vgl_plane_3d<double> proj_plane) { proj_plane_ = proj_plane; }

  void select_proj_plane();

  void move_ground_plane(vgl_plane_3d<double> master_plane,
                         vsol_point_2d_sptr new_pt);

#if 0  //replaced by translate along optical cone (remove after testing)
  //: Translate *this projection plane
  void translate_along_optical_axis(double da);
#endif
  //: Translate/scale according to the optical cone
  // The object vertices are adjusted so that the projection in
  // the master frame is invariant to the motion. This method is
  // useful for perpective cameras where object scale changes with distance
  // from the center of projection
  void translate_along_optical_cone(double da);

  vgl_plane_3d<double> get_proj_plane() const { return proj_plane_; }

  void set_ground_plane(double x1, double y1, double x2, double y2);

  //: if true, makes all the objects selectable, otherwise unselectable
  void set_selection(bool status);

  void backproj_point(vsol_point_2d_sptr p2d, vsol_point_3d_sptr& p3d);

  void backproj_point(vsol_point_2d_sptr p2d,
                      vsol_point_3d_sptr& p3d,
                      vgl_plane_3d<double> proj_plane);

  //: Special case of backprojecting onto the projection plane
  virtual void backproj_poly(vsol_polygon_2d_sptr poly2d,
                             vsol_polygon_3d_sptr& poly3d) {
    backproj_poly(poly2d, poly3d, proj_plane_);
  }

  //: Special case of backprojecting onto the projection plane translated by dist in the direction of the normal
  void backproj_poly(vsol_polygon_2d_sptr poly2d,
                     vsol_polygon_3d_sptr& poly3d,
                     double dist);

  virtual void proj_point(vgl_point_3d<double> world_pt,
                          vgl_point_2d<double> &image_pt);

  virtual void proj_line(vsol_line_3d_sptr line_3d,
                         vsol_line_2d_sptr &line_2d);

  virtual void proj_poly(vsol_polygon_3d_sptr poly3d,
                         vsol_polygon_2d_sptr& poly2d);

#if 0
  void proj_poly(vcl_vector<bmsh3d_vertex*> verts,
                 vcl_vector<vgl_point_2d<double> > &projections);
#endif

  bool intersect(float x1, float y1, float x2, float y2);

  bool intersect(bwm_observable_sptr obj, unsigned face_id,
                 float x1, float y1, float x2, float y2);

  bool intersect(bwm_observable_sptr obj, float img_x, float img_y,
                 unsigned face_id, vgl_point_3d<double> &pt3d);

  bool find_intersection_point(vgl_point_2d<double> img_point,
                               vsol_polygon_3d_sptr poly3d,
                               vgl_point_3d<double>& point3d);

  virtual void camera_center(vgl_point_3d<double> & /*center*/) {}

  bool corr_pt(vgl_point_2d<double> &p) const
  { if (corr_.size()>0) { p = corr_[corr_.size()-1].first; return true; } else return false; }

  //virtual vgl_vector_3d<double> camera_direction(vgl_point_3d<double> origin)=0;

  void extrude_face();

  void extrude_face(vsol_point_2d_sptr pt);

  void divide_face(bwm_observable_sptr obs, unsigned face_id,
                   float x1, float y1, float x2, float y2);

  void scan_regions();

  void load_mesh();

  void load_mesh_multiple();

  void save();
  void save(vcl_string path);
  void save_all();
  void save_all(vcl_string path);

  void triangulate_meshes();

  void move_corr_point(vsol_point_2d_sptr new_pt);

  void set_corr_to_vertex();

  void world_pt_corr();

  void show_geo_position();

  //: displays position at vertex as text, either geo or local coordinates
  void position_vertex(bool show_as_geo = true);

  void scroll_to_point(double lx, double ly, double lz);

  void create_terrain();

  void create_circular_polygon(vcl_vector< vsol_point_2d_sptr > ps_list,
                               vsol_polygon_3d_sptr &circle,
                               int num_sect, double &r, vgl_point_2d<double> &c);

  void set_draw_mode(BWM_DRAW_MODE mode) { mode_=mode; update_all(); }
  void set_mesh_mode() { mode_ = bwm_observer_vgui::MODE_MESH; update_all(); }
  void set_face_mode() { mode_ = bwm_observer_vgui::MODE_POLY; update_all(); }
  void set_edge_mode() { mode_ = bwm_observer_vgui::MODE_EDGE; update_all(); }
  void set_vertex_mode() { mode_ = bwm_observer_vgui::MODE_VERTEX; update_all(); }

  virtual vcl_ostream& print_camera(vcl_ostream& s) { return s; }

  static void project_meshes(vcl_vector<vcl_string> paths,
                             vpgl_camera<double>* cam,
                             vcl_vector<vgl_polygon<double> > &poly_2d_list);

#if 0
  void create_boxm_scene();
  void load_boxm_scene();
#endif
  // ==================  camera calibration methods ================
  void set_horizon();
  void set_focal_length(double focal_length){focal_length_ = focal_length;}
  void set_cam_height(double cam_height){cam_height_ = cam_height;}
  void calibrate_cam_from_horizon();
  void toggle_cam_horizon();
  //=====================  depth map methods ========================
  void set_depth_map_scene(depth_map_scene const& scene){scene_ = scene;}
  void set_ground_plane();
  void set_sky();
  void add_vertical_depth_region(double min_depth, double max_depth,
                                 vcl_string name);
  void save_depth_map_scene(vcl_string const& path);
  void display_depth_map_scene();
  vcl_vector<depth_map_region_sptr> scene_regions();
  void set_ground_plane_max_depth();
 protected:

  //: to compute direction of sun.
  double sun_elev_angle_;
  double sun_azim_angle_;
  bool shadow_mode_;

  vcl_vector<bgui_vsol_soview2D_line_seg*> shadow_line_segs_;

  vpgl_camera<double> *camera_;

  vcl_string cam_path_;

  bool cam_adjusted_;

  vgl_plane_3d<double> proj_plane_;

  //: face that is involved with moving
  vgl_plane_3d<double> moving_face_plane_;

  //: controls extrude for keyboard manipulation
  bool extrude_mode_;
  bwm_observable_sptr extrude_obj_;

  bool show_geo_position_;

  //: list of selected soview objects after the last deselect_all
  vcl_vector<vgui_soview*> selected_soviews_;

  bool geo_position(double u, double v, double& x, double& y, double& z);

  bwm_observer_cam() {}

  virtual bool intersect_ray_and_plane(vgl_point_2d<double> /*img_point*/,
                                       vgl_plane_3d<double> /*plane*/,
                                       vgl_point_3d<double>& /*world_point*/)
  { vcl_cout << "ERROR!  USING CAM OBSERVER's intersect_ray_and_plane"; return false; }

  bool intersect_ray_and_box(vgl_box_3d<double> box,
                             vgl_point_2d<double> img_point,
                             vgl_point_3d <double> &point);

  bool backproj_poly(vsol_polygon_2d_sptr poly2d,
                     vsol_polygon_3d_sptr& poly3d,
                     vgl_plane_3d<double> proj_plane);

  bool find_intersection_points(vgl_point_2d<double> const img_point1,
                                vgl_point_2d<double> const img_point2,
                                vsol_polygon_3d_sptr poly3d,
                                vgl_point_3d<double>& point1,
                                vgl_point_3d<double>& l1,
                                vgl_point_3d<double>& l2,
                                vgl_point_3d<double>& point2,
                                vgl_point_3d<double>& l3,
                                vgl_point_3d<double>& l4);

  unsigned find_index_of_v(bwm_soview2D_vertex* vertex, bgui_vsol_soview2D_polygon* polygon);

  void make_object_selectable(bwm_observable_sptr obj, bool status);

  //: deselects all the selected objects and sets their styles back to mesh
  void deselect();
  //: objects for camera calibration
  double focal_length_;
  double cam_height_;
  vsol_line_2d_sptr horizon_;//set manually
  vgui_soview* horizon_soview_;//computed from camera
  //: objects for depth map
  depth_map_scene scene_;
};

#endif
