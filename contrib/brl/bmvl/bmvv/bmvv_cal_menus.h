#ifndef bmvv_cal_menus_h_
#define bmvv_cal_menus_h_
//-----------------------------------------------------------------------------
//:
// \file
// \brief the setup for menus for bmvv
//  This is essentially a namespace for static methods;
//  there is no public constructor
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy December 11, 2002    Initial version.
// \endverbatim
//-----------------------------------------------------------------------------

#include <bmvv/dll.h>
class vgui_menu;

class bmvv_cal_menus
{
 public:
  BMVV_DLL_DATA  static void quit_callback();
  BMVV_DLL_DATA static void load_image_callback();
  BMVV_DLL_DATA static void save_image_callback();
  BMVV_DLL_DATA static void adjust_contrast_callback();
  BMVV_DLL_DATA static void set_range_params_callback();
  BMVV_DLL_DATA static void clear_display_callback();
  BMVV_DLL_DATA static void clear_selected_callback();
  BMVV_DLL_DATA static void compute_ransac_homography_callback();
  BMVV_DLL_DATA static void load_image_and_camera_callback();
  BMVV_DLL_DATA static void load_corrs_callback();
  BMVV_DLL_DATA static void save_corrs_callback();
  BMVV_DLL_DATA static void save_constraints_callback();
  BMVV_DLL_DATA static void load_world_callback();
  BMVV_DLL_DATA static void save_camera_callback();
  BMVV_DLL_DATA static void pick_correspondence_callback();
  BMVV_DLL_DATA static void pick_vertical_callback();
  BMVV_DLL_DATA static void remove_correspondence_callback();
  BMVV_DLL_DATA static void solve_camera_callback();
  BMVV_DLL_DATA static void set_identity_camera_callback();
  BMVV_DLL_DATA static void map_image_to_world_callback();
  BMVV_DLL_DATA static void clear_world_callback();
  BMVV_DLL_DATA static void set_height_callback();
  BMVV_DLL_DATA static void pick_polygon_callback();
  BMVV_DLL_DATA static void confirm_polygon_callback();
  BMVV_DLL_DATA static void create_box_callback();
  BMVV_DLL_DATA static void save_world_callback();
  BMVV_DLL_DATA static void save_world_ply2_callback();
  BMVV_DLL_DATA static void read_world_ply2_callback();
  BMVV_DLL_DATA static void ply2_to_vrml_callback();
  BMVV_DLL_DATA static vgui_menu get_menu();
 private:
  bmvv_cal_menus() {}
};

#endif // bmvv_cal_menus_h_
