//-*- c++ -*-------------------------------------------------------------------
#ifndef vgui_h_
#define vgui_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vgui
// .INCLUDE vgui/vgui.h
// .FILE vgui.cxx
//
// .SECTION Description:
//
// vgui is a namespace with a singleton vgui_toolkit instance
// which acts as an abstract factory. The static methods on
// vgui simply despatch the call to the selected toolkit.
//
// Order of things :
// 1. Registration. Toolkits available to the application are registered
//    in a global list. Registration is done by the constructor of vgui_toolkit,
//    so creating a vgui_toolkit amounts to registration. Sometimes this can be
//    done at library initialization time but sometimes it can't, e.g. for
//    static builds. In that case, a {\em tag function} must be called
//    explicitly. See vgui_tag.*
//
// 2. Choice of toolkit to use. Several toolkits may be available, and so
//    a choice must be made as to which to use. The choice of toolkit is
//    specified with the select() methods.
//
// 3. Initialization of toolkit. There is only one method for doing this, namely
//      vgui::init(argc, argv);
//    which needs a plausible command line. If no toolkit has been selected
//    the init() function will try to choose one for you based on the the
//    given command line.
//
// .SECTION Author:
//              Philip C. Pritchett, 30 Sep 99
//              Robotics Research Group, University of Oxford
//
// .SECTION Modifications:
// 26 Oct 1999  fsm. various changes to facilitate the port of old impl code.
// 30-AUG-2000  Marko Bacic, Oxford RRG -- Added flags to support MFC accel.

#include "dll.h"
#include <vcl_string.h>
#include <vcl_iostream.h> // for the status bar ostream.

class vgui_window;
class vgui_adaptor;
class vgui_event;
class vgui_menu;
class vgui_dialog_impl;
class vgui_tableau;
class vgui_toolkit;
struct vgui_tableau_sptr;

// See vgui.cxx for information about this class.
class vgui {
public:
  // Method for determining if a given toolkit is available.
  static bool exists(char const *toolkit);

  // Method for selecting a specific toolkit. This will abort()
  // if given a toolkit which is not available.
  static void select(char const *toolkit);

  // Method for selecting a toolkit based on command line
  // arguments and environment variables.
  //
  // First, the command line is scanned for --factory=xxx options.
  //
  // If no such option is given, the environment variable 'vgui' is inspected.
  //
  // If no such environment variable is set, no toolkit is selected and the
  // function returns false. Else the return value is true.
  static bool select(int &argc, char **argv);


  // Initialize the selected toolkit, passing it the
  // given command line.
  static void init(int &argc, char **argv);


  // Factory methods
  static vgui_window* produce_window(int width, int height,
                                     vgui_menu const & menubar, vcl_string const &title ="");
  static vgui_window* produce_window(int width, int height, vcl_string const &title ="");

  static vgui_dialog_impl* produce_dialog(vcl_string const &name);

  // Convenience methods
  static int run(vgui_tableau_sptr const&, int w, int h, vcl_string const &title ="");
  static int run(vgui_tableau_sptr const&, int w, int h, vgui_menu const &menubar, vcl_string const &title ="");
  static vgui_window *adapt(vgui_tableau_sptr const&, int w, int h, vcl_string const &title ="");
  static vgui_window *adapt(vgui_tableau_sptr const&, int w, int h, vgui_menu const &, vcl_string const &title ="");

  // Functions for event-loop management
  static int  run();
  static void run_one_event();
  static void run_till_idle();
  static void flush();
  static void add_event(vgui_event const &);
  static void quit(); // quit application.

  // statusbar related
  static vgui_DLLDATA vcl_ostream out;

private:
  // selected toolkit instance.
  static vgui_DLLDATA vgui_toolkit* instance_;
  //
  static vgui_DLLDATA bool init_called;
};

#endif // vgui_h_
