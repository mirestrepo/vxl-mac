#ifndef vidfpl_menus_h_
#define vidfpl_menus_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief vidfpl_menus
//   the setup for vidfpl_menus for the video file player
//
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy Oct 05, 2002    Initial version.
// \endverbatim
//--------------------------------------------------------------------------------
//: Menu callbacks are represented as static methods
//  The key method is ::get_menu, which does the work of
//  assembing the menu bar for the main executable 
class vidfpl_menus
{
 public:
  static void load_video_callback();
  static void play_video_callback();
  static void pause_video_callback();
  static void next_frame_callback();
  static void prev_frame_callback();
  static void stop_video_callback();
  static void easy2D_tableau_demo_callback();
  static void no_op_callback();
  static void difference_frames_callback();
  static void compute_motion_callback();
  static void compute_lucas_kanade_callback();
  static void compute_harris_corners_callback();
  static void compute_vd_edges_callback();
  static void compute_curve_tracking_callback();
  static void quit_callback();
  static vgui_menu get_menu();
 private:
  vidfpl_menus(){};
};
#endif // vidfpl_menus_h_
