// This is oxl/vgui/vgui_function_tableau.h
#ifndef vgui_function_tableau_h_
#define vgui_function_tableau_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author Philip C. Pritchett, Robotics Research Group, University of Oxford
// \date   25 Nov 99
// \brief  Tableau where functions passed in are called for various events.
//
//  Contains classes:  vgui_function_tableau
//
// \verbatim
//  Modifications:
//    25-NOV-1999 P.Pritchett - Initial version.
// \endverbatim

#include <vgui/vgui_tableau.h>
#include "dll.h"

//:  Tableau where functions passed in are called for various events.
//
//   For example, you can pass in my_draw_function() to draw() and
//   it will be called every time a draw event occurs in this tableau.
class vgui_function_tableau : public vgui_tableau
{
 public:
  typedef bool (*function)(const vgui_event&);

  vgui_function_tableau();

  vcl_string type_name() const { return "vgui_function_tableau"; }

  void draw(function f) {draw_ = f;}
  void mouse_up(function f) {mouse_up_ = f;}
  void mouse_down(function f) {mouse_down_ = f;};
  void motion(function f) {motion_ = f;}
  void key_press(function f) {key_press_ = f;}
  void help(function f) {help_ = f;}


  static vgui_DLLDATA bool redraw;

 protected:
 ~vgui_function_tableau();
  bool handle(const vgui_event&);

  function draw_;
  function mouse_up_;
  function mouse_down_;
  function motion_;
  function key_press_;
  function help_;
};

#endif // vgui_function_tableau_h_
