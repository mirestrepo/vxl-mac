// This is oxl/ogui/vgui_event_loop.h
#ifndef ogui_event_loop_h_
#define ogui_event_loop_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author fsm

struct vgui_slab;
struct vgui_tableau_sptr;

void vgui_event_loop_attach(vgui_slab *s, vgui_tableau_sptr const& t);

void vgui_event_loop_detach(vgui_slab *s);

void vgui_event_loop_service();

int  vgui_event_loop();

void vgui_event_loop_finish();

#endif // ogui_event_loop_h_
