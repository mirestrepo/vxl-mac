//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vgui_dialog_impl
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 25 Oct 99
//
//-----------------------------------------------------------------------------

#include "vgui_dialog_impl.h"
#include <vcl_iostream.h>
#include <vcl_cassert.h>
#include <vgui/internals/vgui_dialog_field.h>
#include <vgui/internals/vgui_simple_field.h>
#include <vgui/internals/vgui_string_field.h>
#include <vgui/internals/vgui_file_field.h>

vgui_dialog_impl::vgui_dialog_impl(const char* n) 
  : name(n)
  , cancel_button_text_("Cancel")
  , ok_button_text_("OK")
{
  assert(n);
}


vgui_dialog_impl::~vgui_dialog_impl()
{
}

//--------------------------------------------------------------------------------
// -- Add a boolean field to the dialog box.
void vgui_dialog_impl::bool_field(const char* txt, bool& val) {
  vgui_bool_field *field = new vgui_bool_field(txt, val);
 
  element l;
  l.type = bool_elem;
  l.widget = bool_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}
 
//--------------------------------------------------------------------------------
// -- Add an integer field to the dialog box.
void vgui_dialog_impl::int_field(const char* txt, int& val) {
  vgui_int_field *field = new vgui_int_field(txt, val);
 
  element l;
  l.type = int_elem;
  l.widget = int_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}

//--------------------------------------------------------------------------------
// -- Add a long field to the dialog box.
void vgui_dialog_impl::long_field(const char* txt, long& val) {
  vgui_long_field *field = new vgui_long_field(txt, val);
 
  element l;
  l.type = long_elem;
  l.widget = long_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}
 
//--------------------------------------------------------------------------------
// -- Add a float field to the dialog box.
void vgui_dialog_impl::float_field(const char* txt, float& val) {
  vgui_float_field *field = new vgui_float_field(txt, val);
 
  element l;
  l.type = float_elem;
  l.widget = float_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}
 
//--------------------------------------------------------------------------------
// -- Add a double field to the dialog box.
void vgui_dialog_impl::double_field(const char* txt, double& val) {
  vgui_double_field *field = new vgui_double_field(txt, val);
 
  element l;
  l.type = double_elem;
  l.widget = double_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}

//--------------------------------------------------------------------------------
// -- Add a vcl_string field to the dialog box.
void vgui_dialog_impl::string_field(const char* txt, vcl_string& val) {
 
  vgui_string_field *field = new vgui_string_field(txt, val);
 
  element l;
  l.type = string_elem;
  l.widget = string_field_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}
 
//--------------------------------------------------------------------------------
// -- Add a choice field to the dialog box.
void vgui_dialog_impl::choice_field(const char* txt,
                                          const vcl_vector<vcl_string>& labels, int& val) {
 
  vgui_int_field *field = new vgui_int_field(txt, val);
 
  element l;
  l.type = choice_elem;
  l.widget = choice_field_widget(txt, labels, val);
  l.field = field;
 
  elements.push_back(l);
}

void vgui_dialog_impl::file_browser(const char* txt, vcl_string& regexp, vcl_string& val) {
  vgui_file_field *field = new vgui_file_field(txt, regexp, val);
  
  element l;
  l.type = file_bsr;
  l.widget = file_browser_widget(txt, regexp, val);
  l.field = field;
  
  elements.push_back(l);
}

void vgui_dialog_impl::inline_file_browser(const char *txt,vcl_string & regexp,
vcl_string& val)
{
  vgui_file_field *field = new vgui_file_field(txt, regexp, val);
  
  element l;
  l.type = inline_file_bsr;
  l.widget = inline_file_browser_widget(txt, regexp, val);
  l.field = field;
  
  elements.push_back(l);
}

void vgui_dialog_impl::color_chooser(const char* txt, vcl_string& val) {
  vgui_string_field *field = new vgui_string_field(txt, val);
 
  element l;
  l.type = color_csr;
  l.widget = color_chooser_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}

void vgui_dialog_impl::inline_color_chooser(const char* txt, vcl_string& val) {
  vgui_string_field *field = new vgui_string_field(txt, val);
 
  element l;
  l.type = inline_color_csr;
  l.widget = inline_color_chooser_widget(txt, val);
  l.field = field;
 
  elements.push_back(l);
}


//--------------------------------------------------------------------------------
// -- Add a text message to the dialog box.
void vgui_dialog_impl::text_message(const char* txt) {
  int dummy_int = 0;
  vgui_int_field *field = new vgui_int_field(txt, dummy_int);
 
  element l;
  l.type = text_msg;
  l.widget = text_message_widget(txt);
  l.field = field;
 
  elements.push_back(l);
}

void* vgui_dialog_impl::bool_field_widget(const char*, bool&) {
  return 0;
}
void* vgui_dialog_impl::int_field_widget(const char*, int&) {
  return 0;
}
void* vgui_dialog_impl::long_field_widget(const char*, long&) {
  return 0;
}
void* vgui_dialog_impl::float_field_widget(const char*, float&) {
  return 0;
}
void* vgui_dialog_impl::double_field_widget(const char*, double&) {
  return 0;
}
void* vgui_dialog_impl::string_field_widget(const char*, vcl_string&) {
  return 0;
}
void* vgui_dialog_impl::choice_field_widget(const char*, const vcl_vector<vcl_string>&, int&) {
  return 0;
}
void* vgui_dialog_impl::text_message_widget(const char*) {
  return 0;
}
void* vgui_dialog_impl::file_browser_widget(const char*, vcl_string&, vcl_string&) {
  return 0;
}
void* vgui_dialog_impl::inline_file_browser_widget(const char*, vcl_string&, vcl_string&) {
  return 0;
}
void* vgui_dialog_impl::color_chooser_widget(const char* txt, vcl_string& val) {
  return string_field_widget(txt, val);
}
void* vgui_dialog_impl::inline_color_chooser_widget(const char* txt, vcl_string& val) {
  return string_field_widget(txt, val);
}
//--------------------------------------------------------------------------------
//-- Changes the modality of the dialog.  True makes the dialog modal (i.e. the
// dialog 'grabs' all events), this is the default.  False makes the dialog non-modal.
void vgui_dialog_impl::modal(const bool) {
  vcl_cerr << "No function defined to change dialog modality, by default dialogs are modal" << vcl_endl;
}

void vgui_dialog_impl::set_cancel_button(char const* txt)
{
  cancel_button_text_ = txt ? txt : "";
}

void vgui_dialog_impl::set_ok_button(char const* txt)
{
  ok_button_text_ = txt ? txt : "";
}
