/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vgui_window.h"
#include <vcl_iostream.h>
#include <vgui/vgui_adaptor.h>
#include <vgui/vgui_macro.h>
#include <vgui/vgui_statusbar.h>

//--------------------------------------------------------------------------------

vgui_window::vgui_window() {
}

vgui_window::~vgui_window() {
}

//--------------------------------------------------------------------------------

void vgui_window::set_menubar(vgui_menu const &) {
  vgui_macro_warning << "set_menubar() not implemented for this window" << vcl_endl;
}

//--------------------------------------------------------------------------------

void vgui_window::set_adaptor(vgui_adaptor*) {
  vgui_macro_warning << "set_adaptor() not implementated for this window" << vcl_endl;
}

vgui_adaptor* vgui_window::get_adaptor() {
  vgui_macro_warning << "get_adaptor() not implementated for this window" << vcl_endl;
  return 0;
}

vgui_statusbar* vgui_window::get_statusbar() {
  vgui_macro_warning << "get_statusbar() not implementated for this window" << vcl_endl;
  return 0;
}
  
//--------------------------------------------------------------------------------


void vgui_window::show() {
  vgui_macro_warning << "vgui_window::show() dummy implementation" << vcl_endl;
}
void vgui_window::enable_hscrollbar(bool) {
  vgui_macro_warning << "vgui_window::enable_hscrollbar() dummy implementation" << vcl_endl;
}
void vgui_window::enable_vscrollbar(bool) {
  vgui_macro_warning << "vgui_window::enable_vscrollbar() dummy implementation" << vcl_endl;
}

void vgui_window::hide() {
  vgui_macro_warning << "vgui_window::show() dummy implementation" << vcl_endl;
}

void vgui_window::iconify() {
  vgui_macro_warning << "vgui_window::hide() dummy implementation" << vcl_endl;
}

void vgui_window::reshape(unsigned,unsigned) {
  vgui_macro_warning << "vgui_window::reshape() dummy implementation" << vcl_endl;
}

void vgui_window::reposition(int,int) {
  vgui_macro_warning << "vgui_window::reposition() dummy implementation" << vcl_endl;
}

void vgui_window::set_title(vcl_string const&) {
  vgui_macro_warning << "vgui_window::set_title() dummy implementation" << vcl_endl;
}

//--------------------------------------------------------------------------------
