#ifndef vgui_mfc_dialog_impl_h_
#define vgui_mfc_dialog_impl_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME vgui_mfc_dialog_impl - Provides support for a dialog box
// .LIBRARY vgui-mfc
// .HEADER vxl Package
// .INCLUDE vgui/impl/mfc/vgui_mfc_dialog_impl.h
// .FILE vgui_mfc_dialog_impl.cxx
//
// .SECTION Description:
//
//   Specialization of vgui_dialog_impl for mfc. Creates a mfc dialog box.
//   Based on vgui_gtk_dialog_impl
//
// .SECTION Author:
//              Marko Bacic, 31 Jul 2000
//              Robotics Research Group, University of Oxford
//
// .SECTION Modifications:
//-----------------------------------------------------------------------------

#include <vgui/internals/vgui_dialog_impl.h>
#include "stdafx.h"
#define MAX_ITEMS 255
// MFC documentation says that our IDs should be in the range 0x8000-0xDFF
#define ID_BROWSE_FILES 0x8000  // Assume that there won't be more than 100 browser buttons
#define ID_CHOOSE_COLOUR 0x8100 // Assume that there won't be more than 100 color chooser buttons
#define ID_EDIT 0x8200
#define ID_COMBOBOX 0x8300

class vgui_mfc_dialog_impl : public CWnd,public vgui_dialog_impl {
public:
  vgui_mfc_dialog_impl(const char* name);
  ~vgui_mfc_dialog_impl();

  void* choice_field_widget(const char*, const vcl_vector<vcl_string>&, int&);
  //: Sets the modality of the dialog box.
  void modal(const bool);
  //: Display the dialog box.
  bool ask();
protected:
  LOGFONT m_logfont;
  //: Called by MFC when the user clicks the OK button.
  virtual void OnOk();
  //: Called by MFC when the user clicks the cancel button.
  virtual void OnCancel();
  //: Called by MFC when the user clicks the (file) browse button.
  virtual void OnBrowse(UINT uID);
  //: Called by MFC when the user clicks the colour chooser button.
  virtual void OnChooseColour(UINT uID);
  //: Called by MFC when the appication is about to terminate.
  afx_msg void OnClose();
private:
  int nResult;
  //: File browser counter.
  int count_fbsr;
  //: Colour chooser counter.
  int count_csr;
  bool ok_clicked;
  //: Array of MFC file browser objects.
  CWnd *fbsrs[100];
  //: Array of MFC colour chooser objects.
  CWnd *csrs[100];
  //: MFC dialog box object.
  CWnd *dialog_box;
  vcl_vector<CWnd *> wlist;
  //: List of created MFC objects (so we can delete them).
  vcl_vector<CWnd *> awlist;
  DECLARE_MESSAGE_MAP()
};

#endif // vgui_mfc_dialog_impl_h_
