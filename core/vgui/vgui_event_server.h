// This is ./oxl/vgui/vgui_event_server.h

//:
// \file
// \author Philip C. Pritchett, Robotics Research Group, University of Oxford
// \date   11 Sep 99
// \brief  Procedural event grabbing from a tableau
//
// \verbatim
//  Modifications:
//    11-SEP-1999 P.Pritchett - Initial version.
//    26-APR-2002 K.Y.McGaul - Converted to and added doxygen style comments.
// \endverbatim

#ifndef vgui_event_server_h_
#define vgui_event_server_h_
#ifdef __GNUC__
#pragma interface
#endif

#include <vgui/vgui_event.h>
#include <vgui/vgui_menu.h>
#include <vgui/vgui_tableau.h>
class vgui_event_server_interpose_tableau;

//: Procedural event grabbing from a tableau
//
//  vgui_event_server allows you to grab events intended for a tableau in a
//  procedural way, (i.e. in a "for" loop), in order to temporarily take control
//  of a tableau's event stream.  This is useful for modal operations such as
//  picking (- or maybe not, see below).
//
//  WARNING: This class is not as useful as it first appears!!
//   If we look at events outside the handle function of a tableau (e.g. by
//   using a vgui_event_server) then the gl state associated with those events 
//   will have changed.  This means for a draw_overlay event we would end up 
//   drawing into the wrong buffer. For a mouse event we may not be able to get 
//   the position in the image using the projection_inspector (if e.g. the image 
//   was zoomed) since all the gl matrices would have been reset - kym.
class vgui_event_server {
public:
  vgui_event_server(vgui_tableau_sptr const&);

  void reset();
  bool next();

  void set_popup(const vgui_menu&);

  vgui_event last_event();

protected:
 ~vgui_event_server();

private:

  friend class vgui_event_server_interpose_tableau;
  vgui_event_server_interpose_tableau* grabber_;
  vgui_tableau_sptr grabber_reference_;
  vgui_event last_event_;
  bool use_event_;
  vgui_menu menu;
};

#endif // vgui_event_server_h_
