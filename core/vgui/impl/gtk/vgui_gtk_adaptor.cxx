// This is oxl/vgui/impl/gtk/vgui_gtk_adaptor.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author Philip C. Pritchett, RRG, University of Oxford
// \date   19 Dec 99
// \brief  See vgui_gtk_adaptor.h for a description of this file.
// 
// \verbatim
//  Modifications:
//   04-OCT-2002 K.Y.McGaul - Use event.set_key() to set key for events,
//                            makes all key chars lower case.
//                          - Set event.ascii_char to actual key stroke
// \endverbatim

#include "vgui_gtk_adaptor.h"
#include <vcl_cstdlib.h>
#include <vcl_cassert.h>
#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>
#include <gtkgl/gtkglarea.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_popup_params.h>
#include <vgui/internals/vgui_overlay_helper.h>
#include "vgui_gtk_utils.h"
#include "vgui_gtk_window.h"

static bool debug = false;
vgui_menu vgui_gtk_adaptor::last_popup;

static gint timeout_callback(gpointer);

//--------------------------------------------------------------------------------
//: Constructors
vgui_gtk_adaptor::vgui_gtk_adaptor(vgui_gtk_window* win) : win_(win), ovl_helper(0), last_mouse_x(0), last_mouse_y(0) {

  widget = gtk_gl_area_new_vargs(0/*NULL*/,         // no sharing
                                 GDK_GL_RGBA,
                                 GDK_GL_DOUBLEBUFFER,
                                 GDK_GL_RED_SIZE, 8,
                                 GDK_GL_GREEN_SIZE, 8,
                                 GDK_GL_BLUE_SIZE, 8,
                                 //GDK_GL_ALPHA_SIZE, 8,
                                 GDK_GL_DEPTH_SIZE,1,
                                     GDK_GL_NONE); // last argument must be GDK_GL_NONE
  if (!widget) {
    widget = gtk_gl_area_new_vargs(0/*NULL*/,         // no sharing
                                   GDK_GL_RGBA,
                                   GDK_GL_DOUBLEBUFFER,
                                   GDK_GL_RED_SIZE, 5,
                                   GDK_GL_GREEN_SIZE, 6,
                                   GDK_GL_BLUE_SIZE, 5,
                                   GDK_GL_DEPTH_SIZE,1,
                                   GDK_GL_NONE); // last argument must be GDK_GL_NONE
  }

  if (!widget) {
    widget = gtk_gl_area_new_vargs(0/*NULL*/,         // no sharing
                                   GDK_GL_RGBA,
                                   GDK_GL_DOUBLEBUFFER,
                                   GDK_GL_DEPTH_SIZE,1,
                                   GDK_GL_NONE); // last argument must be GDK_GL_NONE
  }

  if (!widget) {
    vcl_cerr << __FILE__ << " : Could not get a GL visual!\n";
    vcl_abort();
  }

  gtk_widget_set_events(widget,
                        GDK_EXPOSURE_MASK |
                        GDK_POINTER_MOTION_MASK |
                        GDK_POINTER_MOTION_HINT_MASK |
                        GDK_BUTTON_PRESS_MASK |
                        GDK_BUTTON_RELEASE_MASK |
                        GDK_KEY_PRESS_MASK |
                        GDK_KEY_RELEASE_MASK |
                        GDK_ENTER_NOTIFY_MASK |
                        GDK_LEAVE_NOTIFY_MASK);

  event_handler_id = gtk_signal_connect(GTK_OBJECT(widget), "event", GTK_SIGNAL_FUNC(handle), this);

  GTK_WIDGET_SET_FLAGS(widget, GTK_CAN_FOCUS);

  redraw_requested = false;
  destroy_requested = false;
}

//: Destructor
vgui_gtk_adaptor::~vgui_gtk_adaptor() {
  if (ovl_helper)
    delete ovl_helper;
  ovl_helper = 0;

  // No more events must reach the adaptor now.
  gtk_signal_disconnect(GTK_OBJECT(widget), event_handler_id);

  glFlush();
  gtk_widget_destroy(widget);
}


vgui_window* vgui_gtk_adaptor::get_window() const {
  return win_;
}

void vgui_gtk_adaptor::swap_buffers() {
  make_current();
  gtk_gl_area_swapbuffers(GTK_GL_AREA(widget));
}

void vgui_gtk_adaptor::make_current() {
  assert(gtk_gl_area_make_current(GTK_GL_AREA(widget)));
}

void vgui_gtk_adaptor::post_redraw() {
  if (!redraw_requested) {
    redraw_requested = true;
    gtk_idle_add(idle_callback_for_redraw, this);
  }
}

void vgui_gtk_adaptor::post_overlay_redraw() {
  if (!ovl_helper)
    ovl_helper = new vgui_overlay_helper(this);
  ovl_helper->post_overlay_redraw();
}

//: gtk will pass this structure to the timer callback.
typedef struct {
  vgui_gtk_adaptor *adapt;
  int name;
} vgui_gtk_adaptor_callback_data;

void vgui_gtk_adaptor::post_timer(float timeout, int name) {
  vgui_gtk_adaptor_callback_data *cd = new vgui_gtk_adaptor_callback_data; // <*> acquire
  cd->adapt = this;
  cd->name = name;

  gtk_timeout_add(int(timeout*1000), // timeout in milliseconds
                  timeout_callback,
                  cd);
}

void vgui_gtk_adaptor::post_destroy() {
  if (!destroy_requested) {
    destroy_requested = true;
    gtk_idle_add(idle_callback_for_destroy, this);
  }
}

void vgui_gtk_adaptor::set_default_popup(vgui_menu) {
  vcl_cerr << "vgui_gtk_adaptor::set_default_popup\n";
}

vgui_menu vgui_gtk_adaptor::get_popup() {
  vcl_cerr << "vgui_gtk_adaptor::get_popup\n";
  return vgui_menu();
}

gint vgui_gtk_adaptor::handle(GtkWidget *widget,
                              GdkEvent *gev,
                              gpointer context) {

  vgui_gtk_adaptor* adaptor = (vgui_gtk_adaptor*) context;
  adaptor->make_current();

  bool ret_value = TRUE;
  if (vgui_gtk_utils::is_modifier(gev))
    ret_value = FALSE;

  vgui_event event;

  GdkEventType type = gev->type;

  if (type==GDK_EXPOSE || type==GDK_MAP) {
    adaptor->draw();
    return TRUE;
  }
  else if (type==GDK_CONFIGURE) {
    adaptor->reshape();
    return TRUE;
  }
  else if (type==GDK_MOTION_NOTIFY) {
    event.type = vgui_MOTION;
    GdkEventMotion *e = (GdkEventMotion*)gev;
    if (e->is_hint) {
      int x,y;
      GdkModifierType state;
      gdk_window_get_pointer(e->window, &x, &y, &state);
      vgui_gtk_utils::set_modifiers(event, state);
      vgui_gtk_utils::set_coordinates(event, x, y);
    } else {
      vgui_gtk_utils::set_modifiers(event,e->state);
      vgui_gtk_utils::set_coordinates(event,e->x, e->y);
    }
    adaptor->last_mouse_x = event.wx;
    adaptor->last_mouse_y = event.wy;
  }
  else if (type==GDK_BUTTON_PRESS) {
    event.type = vgui_BUTTON_DOWN;
    GdkEventButton *e = (GdkEventButton*)gev;
    event.button = vgui_gtk_utils::translate_button(e->button);
    vgui_gtk_utils::set_modifiers(event,e->state);
    vgui_gtk_utils::set_coordinates(event,e->x, e->y);
    adaptor->last_mouse_x = event.wx;
    adaptor->last_mouse_y = event.wy;
  }
  else if (type==GDK_BUTTON_RELEASE) {
    event.type = vgui_BUTTON_UP;
    GdkEventButton *e = (GdkEventButton*)gev;
    event.button = vgui_gtk_utils::translate_button(e->button);
    vgui_gtk_utils::set_modifiers(event,e->state);
    vgui_gtk_utils::set_coordinates(event,e->x, e->y);
    adaptor->last_mouse_x = event.wx;
    adaptor->last_mouse_y = event.wy;
  }
  else if (type==GDK_KEY_PRESS) {
    event.type = vgui_KEY_PRESS;
    GdkEventKey *e = (GdkEventKey*)gev;
    event.set_key( vgui_gtk_utils::translate_key(e));
    event.ascii_char = vgui_gtk_utils::translate_key(e);
    vgui_gtk_utils::set_modifiers(event,e->state);
    event.wx = adaptor->last_mouse_x;
    event.wy = adaptor->last_mouse_y;
  }
  else if (type==GDK_KEY_RELEASE) {
    event.type = vgui_KEY_RELEASE;
    GdkEventKey *e = (GdkEventKey*)gev;
    event.set_key( vgui_gtk_utils::translate_key(e));
    event.ascii_char = vgui_gtk_utils::translate_key(e);
    vgui_gtk_utils::set_modifiers(event,e->state);
    event.wx = adaptor->last_mouse_x;
    event.wy = adaptor->last_mouse_y;
  }
  else if (type==GDK_ENTER_NOTIFY) {
    event.type = vgui_ENTER;
    gtk_widget_grab_focus(GTK_WIDGET(widget));
  }
  else if (type==GDK_LEAVE_NOTIFY) {
    event.type = vgui_LEAVE;
  }
  else {
    event.type = vgui_OTHER;
  }

  if (event.type == vgui_BUTTON_DOWN &&
      event.button == adaptor->popup_button &&
      event.modifier == adaptor->popup_modifier) {

    GdkEventButton *bevent = (GdkEventButton *)gev;

    GtkWidget *popup_menu = gtk_menu_new ();    /* Don't need to show menus */

    vgui_popup_params params;
    params.x = event.wx;
    params.y = event.wy;

    // fsm - assign the popup menu to 'last_popup' to ensure the
    // commands stay in scope for the lifetime of the gtk popup.
    adaptor->last_popup = adaptor->get_total_popup(params);

    vgui_gtk_utils::set_menu(popup_menu, adaptor->last_popup, false);
    gtk_menu_popup(GTK_MENU(popup_menu), 0/*NULL*/, 0/*NULL*/, 0/*NULL*/, 0/*NULL*/,
                   bevent->button, bevent->time);
    return TRUE;
  }

  if (debug) vcl_cerr << "event " << event << vcl_endl;
  if (adaptor->ovl_helper)
    adaptor->ovl_helper->dispatch(event);
  else
    adaptor->dispatch_to_tableau(event);

  return ret_value;
}


void vgui_gtk_adaptor::reshape() {

  make_current();

  width = widget->allocation.width;
  height = widget->allocation.height;

  if (ovl_helper)
    ovl_helper->dispatch(vgui_RESHAPE);
  else
    dispatch_to_tableau(vgui_RESHAPE);
}


//--------------------------------------------------------------------------------
//: This is overriding the gtk draw() method.
void vgui_gtk_adaptor::draw() {
  if (debug) vcl_cerr << "vgui_gtk_adaptor::draw\n";
  make_current();
  glDrawBuffer(GL_BACK);
  if (ovl_helper)
    ovl_helper->dispatch(vgui_DRAW);
  else {
    dispatch_to_tableau(vgui_DRAW);
    swap_buffers();
  }
}

gint vgui_gtk_adaptor::idle_callback_for_redraw(gpointer data) {
  vgui_gtk_adaptor *adaptor = static_cast<vgui_gtk_adaptor*>(data);

  adaptor->draw();

  adaptor->redraw_requested = false;

  // capes - returning FALSE automagically cancels this callback
  return FALSE;
}

// Callback setup by post_destroy. First notifies tableau of the impending
// destruction. Then deletes the adaptor and its associated window.
gint vgui_gtk_adaptor::idle_callback_for_destroy(gpointer data) {
  vgui_gtk_adaptor *adaptor = static_cast<vgui_gtk_adaptor*>(data);

  adaptor->dispatch_to_tableau(vgui_DESTROY);

  vgui_window* win = adaptor->get_window();

  // The adaptor destuctor unrefs its tableau and disconnects/destroys
  // its glarea widget.
  delete adaptor;

  // If we know the parent window then delete it now.
  if (win)
    delete win;
  else
    vcl_cerr << __FILE__ " : parent vgui_gtk_window is unknown, so cannot destroy!\n";

  // capes - returning FALSE automagically cancels this callback
  return FALSE;
}

gint timeout_callback(gpointer data) {
  vgui_gtk_adaptor_callback_data* cd = static_cast<vgui_gtk_adaptor_callback_data*> (data);
  vgui_event e(vgui_TIMER);
  e.timer_id = cd->name;
  cd->adapt->dispatch_to_tableau(e);

  delete cd;                             // <*> release

  // capes - returning FALSE automagically cancels this callback
  return FALSE;
}
