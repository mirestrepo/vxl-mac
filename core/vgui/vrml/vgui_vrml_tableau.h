#ifndef vgui_vrml_tableau_h_
#define vgui_vrml_tableau_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME vgui_vrml_tableau
// .LIBRARY vgui-vrml
// .HEADER vxl Package
// .INCLUDE vgui/vgui/vgui_vrml_tableau.h
// .FILE vgui_vrml_tableau.cxx
//
// .SECTION Author:
//              Philip C. Pritchett, 17 Sep 99
//              Robotics Research Group, University of Oxford
//
// .SECTION Modifications:
//     <none yet>
//
//-----------------------------------------------------------------------------

#include <vgui/vgui_tableau.h>

#include <vgui/vgui_tableau_sptr.h>

class vgui_vrml_tableau;
typedef vgui_tableau_sptr_t<vgui_vrml_tableau> vgui_vrml_tableau_sptr;

class QvVrmlFile;
class vgui_vrml_draw_visitor;

class vgui_vrml_tableau : public vgui_tableau
{
public:
  vgui_vrml_tableau(const char* file, bool scale);
  ~vgui_vrml_tableau();

  // vgui_tableau methods/data
  bool handle(const vgui_event &);
  void invalidate_vrml();

  vcl_string file_name() const;
  vcl_string pretty_name() const;
  vcl_string type_name() const;

  QvVrmlFile* vrml;
  vgui_vrml_draw_visitor* drawer;
  int setup_dl;

  void set_rescale( bool flag) { rescale_model= flag; }

private:

  bool rescale_model;
};


struct vgui_vrml_tableau_new : public vgui_vrml_tableau_sptr {
  typedef vgui_vrml_tableau_sptr base;
  vgui_vrml_tableau_new(char const* file, bool scale = true) : base(new vgui_vrml_tableau(file, scale)) { }
};



#endif // vgui_vrml_tableau_h_
