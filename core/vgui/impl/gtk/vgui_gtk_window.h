#ifndef vgui_gtk_window_h_
#define vgui_gtk_window_h_
#ifdef __GNUC__
#pragma interface
#endif
// 
// .NAME vgui_gtk_window - specialization of vgui_window for GTK
// .LIBRARY vgui-gtk
// .HEADER vxl Package
// .INCLUDE vgui/impl/gtk/vgui_gtk_window.h
// .FILE vgui_gtk_window.cxx
//
// .SECTION Description:
//   Specialization of vgui_window for GTK.
//   Provides functions for manipulating a window.
//
// .SECTION Author:
//              Philip C. Pritchett, 18 Dec 99
//              Robotics Research Group, University of Oxford
//
// .SECTION Modifications
//  13-JUL-00   Marko Bacic, Oxford RRG -- Added support for menu shortcuts
//  21-SEP-00   capes@robots -- Changed destructor to actually destroy the widgets
//                              Facilitates the post_destroy() adaptor method.
//-----------------------------------------------------------------------------

#ifdef __SUNPRO_CC
// <string> breaks if NULL is defined to "(void*)0".
# include <vcl_string.h>
#endif

#include <vgui/vgui_window.h>
#include <vgui/vgui_menu.h>
class vgui_gtk_adaptor;

#include <gtk/gtk.h>
#include "vgui_gtk_statusbar.h"

class vgui_gtk_window : public vgui_window
{
public:
    
  vgui_gtk_window(int w, int h, const vgui_menu& menu, const char* title);
  vgui_gtk_window(int w, int h, const char* title);
 ~vgui_gtk_window();

  bool use_menubar;
  bool use_statusbar;

  void init();
  
  void show();
  void hide();

  void set_menubar(const vgui_menu &menu);
  void set_statusbar(bool) {}
  
  void set_adaptor(vgui_adaptor*);
  vgui_adaptor* get_adaptor();
  vgui_statusbar* get_statusbar() { return &statusbar; }

  // gtk specific
  vgui_gtk_adaptor *adaptor;
  vgui_gtk_statusbar statusbar;

  GtkWidget *window;
  GtkWidget *box;
  GtkWidget *menubar;

private:
  // This is a place to store any menu passed in, so that it doesn't go out of scope
  // while the popup is on screen.
  vgui_menu* last_menubar; // <-- ask fsm about this.
};

#endif // vgui_gtk_window_h_
