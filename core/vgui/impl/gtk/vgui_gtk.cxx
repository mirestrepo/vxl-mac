//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vgui_gtk
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 16 Sep 99
//
//-----------------------------------------------------------------------------

#include "vgui_gtk.h"
#include <vgui/vgui_gl.h> // glFlush()
#include <gtk/gtk.h>
#include "vgui_gtk_window.h"
#include "vgui_gtk_dialog_impl.h"

static bool debug = false;

vgui_gtk* vgui_gtk::instance_ = 0;

vgui_gtk* vgui_gtk::instance() {
  if (instance_ == 0)
    instance_ = new vgui_gtk;

  return instance_;
}

//---------------------------------------------------------------------------------
// -- Default constructor
vgui_gtk::vgui_gtk()
{
}

//--------------------------------------------------------------------------------
// -- Pure virtual function from vgui (this must be implemented).  Returns the 
// name of the GUI toolkit.
vcl_string vgui_gtk::name() const { return "gtk"; }


//--------------------------------------------------------------------------------
// -- Virtual function from vgui.  Initialise the implementation of vgui.
void vgui_gtk::init(int &argc, char **argv) {
  if (debug) vcl_cerr << "vgui_gtk::init()" << vcl_endl;
  gtk_init(&argc, &argv);
}


//--------------------------------------------------------------------------------
// -- Virtual function from vgui.   Runs the event loop.
void vgui_gtk::run() {
  if (debug) vcl_cerr << "vgui_gtk::run()" << vcl_endl;
  gtk_main();
}

void vgui_gtk::run_one_event() {
  gtk_main_iteration();
}

void vgui_gtk::run_till_idle() {
  while (gtk_events_pending()) {
    gtk_main_iteration();
    glFlush();
  }
}

void vgui_gtk::flush() {
  glFlush();
  run_till_idle();
}

void vgui_gtk::quit() {
  vcl_cerr << __FILE__ " : terminating GTK+ event loop" << vcl_endl;
  gtk_main_quit(); // capes@robots -- causes the gtk event loop to return
}

//--------------------------------------------------------------------------------
// -- Virtual function from vgui.  Creates a new window with a menubar.
vgui_window* vgui_gtk::produce_window(int width, int height, const vgui_menu& menubar,
					   const char* title) {
  return new vgui_gtk_window(width, height, menubar, title);
}


//--------------------------------------------------------------------------------
// -- Virtual function from vgui.  Creates a new window.
vgui_window* vgui_gtk::produce_window(int width, int height,
					   const char* title) {
  
  return new vgui_gtk_window(width, height, title);
}


//--------------------------------------------------------------------------------
// -- Virtual function from vgui.  Creates a new dialog box.
vgui_dialog_impl* vgui_gtk::produce_dialog(const char* name) {
  return new vgui_gtk_dialog_impl(name);
}
