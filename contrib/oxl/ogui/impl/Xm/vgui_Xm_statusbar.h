// This is oxl/ogui/impl/Xm/vgui_Xm_statusbar.h
#ifndef vgui_Xm_statusbar_h_
#define vgui_Xm_statusbar_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author
//              Philip C. Pritchett, 21 Nov 99
//              Robotics Research Group, University of Oxford
//-----------------------------------------------------------------------------

#include <vcl_string.h>
#include <vcl_iosfwd.h>
#include <X11/Intrinsic.h>
#include <vgui/vgui_statusbuf.h>
#include <vgui/vgui_statusbar.h>

class vgui_Xm_statusbar : public vgui_statusbar
{
 public:

  // Constructors/Destructors--------------------------------------------------

  vgui_Xm_statusbar();
 ~vgui_Xm_statusbar();

  int write(const char* text, int n);
  int write(const char* text);

  Widget widget;
  vcl_string linebuffer;
  vgui_statusbuf* statusbuf;
  vcl_ostream out;
};

#endif // vgui_Xm_statusbar_h_
