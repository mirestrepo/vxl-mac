//-*- c++ -*-------------------------------------------------------------------
//
// This is vgui/impl/mfc/vgui_mfc_doc.h

//:
// \file
// \author  Marko Bacic, Oxford RRG
// \date    27 July 2000
// \brief  Defines a main application document
// \verbatim
//  Modifications:
//    Marko Bacic   27-JUL-2000   Initial version.
// \endverbatim
//

#ifndef AFX_VGUI_MFC_DOC_H__AB1D93B0_8FC7_4E71_ABD0_FA4AA666D5CD__INCLUDED_
#define AFX_VGUI_MFC_DOC_H__AB1D93B0_8FC7_4E71_ABD0_FA4AA666D5CD__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class vgui_tableau;


class vgui_mfc_doc : public CDocument
{
protected: // create from serialization only
  vgui_mfc_doc();
  DECLARE_DYNCREATE(vgui_mfc_doc)

// Attributes
public:

// Operations
public:
 vgui_tableau* get_tableau() { return tableau; }
// Overrides
  // ClassWizard generated virtual function overrides
  //{{AFX_VIRTUAL(vgui_mfc_doc)
  public:
  virtual void Serialize(CArchive& ar);
  //}}AFX_VIRTUAL


// Implementation
public:
  virtual ~vgui_mfc_doc();
#ifdef _DEBUG
  virtual void AssertValid() const;
  virtual void Dump(CDumpContext& dc) const;
#endif

protected:
  // awf: we store our main tableau in the document -- the
  // adaptor(s) view this tableau.
  vgui_tableau* tableau;

};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // AFX_VGUI_MFC_DOC_H__AB1D93B0_8FC7_4E71_ABD0_FA4AA666D5CD__INCLUDED_
