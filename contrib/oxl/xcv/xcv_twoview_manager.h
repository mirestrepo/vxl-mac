#ifndef xcv_twoview_manager_h_
#define xcv_twoview_manager_h_

//--------------------------------------------------------------------------------
// .NAME    xcv_twoview_manager
// .INCLUDE xcv/xcv_twoview_manager.h
// .FILE    xcv_twoview_manager.cxx
// .SECTION Description:
//    Handles events which occur in one view but are displayed in two views
//    simultaneously (eg. displaying the epipolar line in one view corresponding
//    to a mouse press in the other view.)
//    Also holds data which links two views, eg. corner matches, FMatrix and
//    HMatrix2D.
// .SECTION Author
//   K.Y.McGaul
// .SECTION Modifications:
//   K.Y.McGaul     05-MAY-2000    Initial version.
//   Marko Bacic    18-AUG-2000    Sorted out display of epipolar lines
//--------------------------------------------------------------------------------

#include <mvl/FMatrix.h>
#include <mvl/HMatrix2D.h>
#include <mvl/PairMatchSetCorner.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_rubberbander.h>
#include <vgui/vgui_easy2D.h>
#include "xcv_mview_manager.h"

class vgui_event;

class xcv_twoview_manager : public xcv_mview_manager
{
public:

  //: Constructor.
  xcv_twoview_manager();
  //: Destructor.
  ~xcv_twoview_manager();

  //: Set the tableau at the given position to be the given tableau.
  void set_tableau(vgui_tableau_sptr const& tab, unsigned tab_position) ;
  //: Set f_matrix to the given value.
  void set_f_matrix(FMatrix* fm){f_matrix = fm;}
  //: Set h_matrix to the given value.
  void set_h_matrix(HMatrix2D* hm){h_matrix = hm;}
  //: Set corner_matches to the given value.
  void set_corner_matches(PairMatchSetCorner* pmsc){corner_matches = pmsc;}

  //: Toggle between displaying and not displaying the FMatrix.
  void toggle_f_matrix_display();
  //: Toggle between displaying and not displaying the HMatrix2D.
  void toggle_h_matrix_display();
  //: Toggle between displaying and not displaying corner matches.
  void toggle_corner_match_display();

  //: Return f_matrix.
  FMatrix* get_f_matrix(){return f_matrix;}
  //: Return h_matrix.
  HMatrix2D* get_h_matrix(){return h_matrix;}
  //: Return corner_matches.
  PairMatchSetCorner* get_corner_matches(){return corner_matches;}

  void handle_tjunction_event(vgui_event const& e, vgui_tableau_sptr const& child_tab);

private:
  FMatrix* f_matrix;
  HMatrix2D* h_matrix;
  PairMatchSetCorner* corner_matches;
  //: The two tableaux managed by this class.
  vgui_tableau_sptr tabs[2];
  //: Rubberbanders for the managed tableaux.
  vgui_rubberbander_sptr rubberbands[2];
  //: Easy2Ds for the managed tableaux.
  vgui_easy2D_sptr easys[2];
  bool f_matrix_is_displayed;
  bool h_matrix_is_displayed;
  bool corner_matches_are_displayed;
  //: Position in tabs where event is displayed but does not occur.
  unsigned transfer_index;
  //: True while mouse is being dragged (the left mouse button is pressed).
  bool dragging;
  float event_coord_x, event_coord_y;
  //: Coordinates of epipolar line.
  float line_coord_a, line_coord_b, line_coord_c;
  float point_coord_x, point_coord_y;
  bool use_overlays;

  //: Draw a point and the transformed epipolar line on the display.
  void draw_f_matrix(vgui_event const& e, vgui_tableau_sptr const& child_tab, bool make_permanent);
  //: Handle overlay redraw event for FMatrix.
  void draw_overlay_f_matrix(vgui_tableau_sptr const& child_tab);
  //: Draw the point and corresponding point computed using the HMatrix2D.
  void draw_h_matrix(vgui_event const& e, vgui_tableau_sptr const& child_tab, bool make_permanent);
  //: Handle overlay redraw event for HMatrix2D.
  void draw_overlay_h_matrix(vgui_tableau_sptr const& child_tab);
  //: Draw matching corners in two views.
  void draw_corner_matches(vgui_event const& e, vgui_tableau_sptr const& child_tab);
  //: Handle overlay redraw event for corner matches.
  void draw_overlay_corner_matches(vgui_tableau_sptr const& child_tab);
};

#endif // xcv_twoview_manager_
