//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// Class: vgui_enhance
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 17 Nov 99
//
//-----------------------------------------------------------------------------

#include "vgui_enhance.h"

#include <vcl_cmath.h>
#include <vcl_iostream.h>

#include <vnl/vnl_matrix.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_matrix_state.h>


vgui_enhance::vgui_enhance()
  : slot1(this)
  , slot2(this)
  , enhancing_(false)
  , size(50)
  , zoom_factor(1.5)
  , enable_key_bindings(false)
{

}

vgui_enhance::vgui_enhance(vgui_tableau_ref const&t)
  : slot1(this,t)
  , slot2(this)
  , enhancing_(false)
  , size(50)
  , zoom_factor(1.5)
  , enable_key_bindings(false)
{

}

vgui_enhance::vgui_enhance(vgui_tableau_ref const&t1, vgui_tableau_ref const&t2)
  : slot1(this,t1)
  , slot2(this,t2)
  , enhancing_(false)
  , size(50)
  , zoom_factor(1.0)
  , enable_key_bindings(false)
{
}

vgui_enhance::~vgui_enhance()
{
}

void vgui_enhance::set_child(vgui_tableau_ref const&t)
{
  slot1 = vgui_slot(this, t);
}

vcl_string vgui_enhance::file_name() const {return slot1->file_name();}
vcl_string vgui_enhance::type_name() const {return "vgui_enhance";}


bool vgui_enhance::handle(const vgui_event& e) {
  
  if (!enhancing_ && e.type == vgui_BUTTON_DOWN && e.button == vgui_LEFT) {
    enhancing_ = true;
    x = (int)e.wx;
    y = (int)e.wy;
    post_redraw();
    return true;
  }
  
  if (enhancing_ && e.type == vgui_BUTTON_UP && e.button == vgui_LEFT) {
    enhancing_ = false;
    post_redraw();
    return true;
  }
  
  if (enhancing_ && e.type == vgui_MOTION) {
    x = (int)e.wx;
    y = (int)e.wy;
    post_redraw();
    return true;
  }
  
  if (enable_key_bindings && e.type == vgui_KEY_PRESS) {
    switch(e.key) {
    case '[':
      size -= 10;
      size = (size <10) ? 10 : size ;
      post_redraw();
      return true;
    case ']':
      size += 10;
      post_redraw();
      return true;
    case '{':
      zoom_factor -= 0.1f;
      vcl_cerr << "enhance : zoom_factor = " << zoom_factor << vcl_endl;
      post_redraw();
      return true;
    case '}':
      zoom_factor += 0.1f;
      vcl_cerr << "enhance : zoom_factor = " << zoom_factor << vcl_endl;
      post_redraw();
      return true;
    default:
      break; // quell warning
    };
  }
  
  if (e.type == vgui_DRAW) {
    // first draw the child
    slot1->handle(e);
    
    if (enhancing_) {
      
      // get original offsets and scales
      vgui_matrix_state ms;
#if defined(VCL_SGI_CC)
      // expect the spurious warning:
      //"vgui_enhance.cxx", line 113: remark(1552): variable "ms" was set but never used
      // here.
      {
	vgui_matrix_state *ptr = &ms;
	ptr = ptr;
      }
#endif
      vnl_matrix<double> M = ms.modelview_matrix();
      float sx = M(0,0);
      float sy = M(0,0);
      float ox = M(0,3);
      float oy = M(1,3);
      
      

      glEnable(GL_SCISSOR_TEST);
      int size_2 = size+size;
      glScissor(x-size, y-size, size_2, size_2);	
      
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      
      glLoadIdentity();
      
      sx *= zoom_factor;
      sy *= zoom_factor;
      
      GLint vp[4]; glGetIntegerv(GL_VIEWPORT,vp);
      float dx = (        x) - ox;
      float dy = (vp[3]-1-y) - oy;
      float tmpx = zoom_factor*dx - dx;
      float tmpy = zoom_factor*dy - dy;
      
      glTranslatef(ox-tmpx, oy-tmpy, 0);
      glScalef(sx, sy, 1);
      
      if (slot2.child()) 
	slot2->handle(e);
      else 
	slot1->handle(e);
      
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
      glDisable(GL_SCISSOR_TEST);
      
    }
    return true;
    
  }
  bool retv = slot1->handle(e);
  if (!retv && slot2)
    retv = slot2->handle(e);

  return retv;
    
}
