/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vgui_adaptor_tableau.h"

#include <vcl_iostream.h>

#include <vgui/vgui_adaptor.h>

// The sole raison d'etre of the adaptor_tableau is to propagate
// tableau posts onto an adaptor.

vgui_adaptor_tableau::vgui_adaptor_tableau(vgui_adaptor *a)
  : adaptor(a)
  , slot(this, 0)
{
  //cerr << "foo_slot = " << slot << endl;
}

vgui_adaptor_tableau::~vgui_adaptor_tableau() 
{
  vcl_cerr << "~vgui_adaptor_tableau(this = " << (void*)this << ")" << vcl_endl;
}

void vgui_adaptor_tableau::post_message(char const *msg, void const *data) 
{ 
  if (adaptor)
    adaptor->post_message(msg, data);
}

void vgui_adaptor_tableau::post_redraw() 
{ 
  if (adaptor)
    adaptor->post_redraw();
}

void vgui_adaptor_tableau::post_overlay_redraw() 
{ 
  if (adaptor)
    adaptor->post_overlay_redraw();
}
