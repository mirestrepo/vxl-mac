#ifndef vgui_gtk_adaptor_h_
#define vgui_gtk_adaptor_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vgui_gtk_adaptor
// .LIBRARY vgui-gtk
// .HEADER vxl Package
// .INCLUDE vgui/impl/gtk/vgui_gtk_adaptor.h
// .FILE vgui_gtk_adaptor.cxx
//
// .SECTION Description:
//   Specialization of vgui_adaptor for GTK. 
//
// .SECTION Author:
//              Philip C. Pritchett, 19 Dec 99
//              Robotics Research Group, University of Oxford

#ifdef __SUNPRO_CC
// <string> breaks if NULL is defined to "(void*)0".
# include <vcl_string.h>
#endif

#include <vgui/vgui_tableau.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/internals/vgui_adaptor_mixin.h>

#include <gtk/gtk.h>

struct vgui_overlay_helper;
class vgui_gtk_window;

class vgui_gtk_adaptor : public vgui_adaptor, public vgui_adaptor_mixin
{
public:
  typedef vgui_adaptor_mixin mixin;

  vgui_gtk_adaptor(vgui_gtk_window* win = 0);
  ~vgui_gtk_adaptor();

  // vgui_adaptor methods
  void swap_buffers();
  void make_current();
  void post_redraw();
  void post_overlay_redraw();
  void post_timer(float,int);
  void post_destroy();  // schedules destruction of parent vgui_window

  unsigned get_width() const {return mixin::width;}
  unsigned get_height() const {return mixin::height;}
  void get_popup_bindings(vgui_modifier &m, vgui_button &b) const
    { m = mixin::popup_modifier; b = mixin::popup_button; }

  void set_default_popup(vgui_menu);
  vgui_menu get_popup();
  
  void draw();
  void reshape();

  // Returns NULL if the empty constuctor was used
  vgui_window* get_window() const;

  // gtk stuff
  GtkWidget *get_glarea_widget() { return widget; }

private:
  // main GDK-to-vgui event despatcher
  static gint handle(GtkWidget *, GdkEvent *, void *);
  int event_handler_id;

  // idle callbacks which service pending redraw/destroy posts
  static gint idle_callback_for_redraw(gpointer data);
  static gint idle_callback_for_destroy(gpointer data);

  // Flags to prevent queuing of multiple redraw/destroy callbacks
  bool redraw_requested;
  bool destroy_requested;

  // pointer to the gtkglarea widget
  GtkWidget *widget;

  // pointer to the window which contains this adaptor
  vgui_gtk_window* win_;

  // pointer to overlay emulation data
  vgui_overlay_helper *ovl_helper;

  // This is a place to store any menu passed in,
  // so that it doesn't go out of scope while the popup is on screen.
  static vgui_menu last_popup;

  // last position where mouse was seen.
  int last_mouse_x, last_mouse_y;
};

#endif // vgui_gtk_adaptor_h_
