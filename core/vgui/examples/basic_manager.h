//this-sets-emacs-to-*-c++-*-mode
#ifndef basic_manager_h_
#define basic_manager_h_
//----------------------------------------------------------------------------
//:
// \file
// \brief An example of a GUI manager class
// \author
//   J.L. Mundy
//
// \verbatim
//  Modifications:
//   J.L. Mundy December 26, 2002    Initial version.
// \endverbatim
//-----------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vil/vil_image.h>
#include <vgui/vgui_image_tableau_sptr.h>
#include <vgui/vgui_wrapper_tableau.h>

class basic_manager : public vgui_wrapper_tableau
{
 public:
  basic_manager();
  ~basic_manager();
  static basic_manager *instance();
  void quit();
  void load_image();
  void init();
  virtual bool handle(vgui_event const &);
 private:

  vil_image img_;
  vgui_image_tableau_sptr itab_; 
  static basic_manager *instance_;
};
#endif // basic_manager_h_
