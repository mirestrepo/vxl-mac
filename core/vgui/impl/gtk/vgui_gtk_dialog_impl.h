#ifndef vgui_gtk_dialog_impl_h_
#define vgui_gtk_dialog_impl_h_
#ifdef __GNUC__
#pragma interface
#endif
// 
// .NAME vgui_gtk_dialog_impl - Undocumented class FIXME
// .LIBRARY vgui-gtk
// .HEADER vxl Package
// .INCLUDE vgui/impl/gtk/vgui_gtk_dialog_impl.h
// .FILE vgui_gtk_dialog_impl.cxx
//
// .SECTION Description:
//
//   Specialization of vgui_dialog_impl for GTK. Creates a GTK dialog box.
//
// .SECTION Author:
//              Philip C. Pritchett, 28 Dec 99
//              Robotics Research Group, University of Oxford
//
// .SECTION Modifications:
//   K.Y.McGaul  25-JAN-00  Moved all field functions to vgui_dialog_impl.
//                          Added choice_field_widget function.
//   Marko Bacic 11-JUL-00  Added support for inline file dialog box
//   Marko Bacic 12-JUL-00  Added support for inline color chooser box
//   Marko Bacic 14-JUL-00  Fixed misalignment of entry boxes 
//   Marko Bacic 20-JUL-00  Fixed bug in inline file dialog box. Now returns
//                          the full pathname
//-----------------------------------------------------------------------------

#include <vgui/internals/vgui_dialog_impl.h>
#include <gtk/gtk.h>

class vgui_gtk_dialog_impl : public vgui_dialog_impl {
public:
  vgui_gtk_dialog_impl(const char* name);
  ~vgui_gtk_dialog_impl();
  
  void* choice_field_widget(const char*, const vcl_vector<vcl_string>&, int&);

  void modal(const bool);
  
  bool ask();
 
private:
  GtkWidget* dialog_window;
  GtkWidget* vbox;

};

#endif // vgui_gtk_dialog_impl_h_
