#ifndef vgui_slot_data_h_
#define vgui_slot_data_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vgui_slot_data
// .INCLUDE vgui/internals/vgui_slot_data.h
// .FILE internals/vgui_slot_data.cxx
// @author fsm@robots.ox.ac.uk

#include <vcl_vector.h>

struct vgui_slot_impl;
struct vgui_slot;
class vgui_tableau;

class vgui_slot_data {
private:
  vcl_vector<vgui_tableau*> parents;
  friend struct vgui_slot_impl;
  friend struct vgui_slot;
};

#endif // vgui_slot_data_h_
