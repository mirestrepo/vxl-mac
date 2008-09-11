// This is brl/bbas/vidl2/examples/vidl2_player_manager.h
#ifndef vidl2_player_manager_h_
#define vidl2_player_manager_h_
//-----------------------------------------------------------------------------
//:
// \file
// \author Matt Leotta
// \brief A sample vidl2 video player
//
//----------------------------------------------------------------------------

#include <vcl_memory.h>
#include <vgui/vgui_wrapper_tableau.h>
#include <vgui/vgui_image_tableau_sptr.h>
#include <vgui/vgui_viewer2D_tableau_sptr.h>
#include <vgui/vgui_window.h>
#include <vidl2/vidl2_istream.h>
#include <vidl2/vidl2_ostream.h>

//: A singleton manager class for playing videos.
//  The purpose this code is demonstrate the use of vidl2.  The vidl2_player
//  loads, displays, and saves (saving not implemented yet) videos.
//
class vidl2_player_manager : public vgui_wrapper_tableau
{
 public:
  vidl2_player_manager();
  ~vidl2_player_manager();
  //: returns the unique instance of vidl2_player_manger
  static vidl2_player_manager *instance();

  //: clean up before the program terminates
  void quit();

  //: height (in pixels) of the video frame
  unsigned get_height() const { return height_; }

  //: width (in pixels) of the video frame
  unsigned get_width() const { return width_; }

  //: open the input video stream
  void open_istream();

  //: open the output video stream
  void open_ostream();

  //: close the input video stream
  void close_istream();

  //: close the output video stream
  void close_ostream();

  //: Pipe the input stream into the output stream
  void pipe_streams();

  //: loop through the frames and display
  void play_video();

  //: stop at the current frame
  void pause_video();

  //: stop playing and return to the first frame
  void stop_video();

  //: pops up a dialog to indicate what frame to go to
  void go_to_frame();

  //: index to the next frame (must be paused)
  void next_frame();

  //: index to the previous frame (must be paused)
  void prev_frame();


  //: get the window of this player
  vgui_window* get_window() { return win_; }

  //: set the window
  void set_window(vgui_window* win) { win_=win; }

  //: tableau handle function
  virtual bool handle(const vgui_event&);

 protected:
  //utility functions
  void init();
  void redraw();

 private:
  //flags
  bool preload_frames_;
  bool play_video_;

  float time_interval_;
  unsigned width_;
  unsigned height_;
  vcl_auto_ptr<vidl2_istream> istream_;
  vcl_auto_ptr<vidl2_ostream> ostream_;
  vgui_window* win_;
  vgui_viewer2D_tableau_sptr v2D_;
  vgui_image_tableau_sptr itab_;

  static vidl2_player_manager *instance_;
};

#endif // vidl2_player_manager_h_
