//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// .NAME vgui_file_field
// Author: 
// Created: 
//
//-----------------------------------------------------------------------------

#include "vgui_file_field.h"

vgui_file_field::vgui_file_field(const char* label, vcl_string& regular_expression, 
  vcl_string& variable_to_modify)
  : vgui_dialog_field(label)
  , var(variable_to_modify)
  , regexp(regular_expression)
{
}

vgui_file_field::~vgui_file_field()
{
}

vcl_string vgui_file_field::current_value() const
{
  return var;
}

bool vgui_file_field::update_value(const vcl_string &s)
{
  var = s;
  return true;
}
