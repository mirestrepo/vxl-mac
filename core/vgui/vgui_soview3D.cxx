//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// Class: vgui_soview3D
// Author: Philip C. Pritchett, RRG, University of Oxford
// Created: 24 Mar 99
//
//-----------------------------------------------------------------------------

#include "vgui_soview3D.h"

#include <vcl_iostream.h>

#include <vgui/vgui_gl.h>
#include <vgui/vgui_style.h>


vgui_soview3D::vgui_soview3D() {
}




//--------------------------------------------------------------------------//

vcl_ostream& vgui_point3D::print(vcl_ostream& s) const {
  s << "[vgui_point3D " << x << "," << y << "," << z << " ";
  s << " "; return vgui_soview3D::print(s) << "]";
}

void vgui_point3D::draw() {
  
  bool lighting = false;
  if (glIsEnabled(GL_LIGHTING)) {
    lighting = true;
    glDisable(GL_LIGHTING);
  }
  

  //glPointSize(style->point_size);
  glBegin(GL_POINTS);
  glVertex3f(x,y,z);
  glEnd();


  if (lighting)
    glEnable(GL_LIGHTING);

}



//--------------------------------------------------------------------------//

vcl_ostream& vgui_lineseg3D::print(vcl_ostream& s) const {
  s << "[vgui_lineseg3D " << x0 << "," << y0 << " -- " << x1 << "," << y1;
  s << " "; return vgui_soview3D::print(s) << "]";
}

void vgui_lineseg3D::draw() {
  //cerr << "line id : " << id << endl;

  bool lighting = false;
  if (glIsEnabled(GL_LIGHTING)) {
    lighting = true;
    glDisable(GL_LIGHTING);
  }

  //glLineWidth(style->line_width);
  glBegin(GL_LINES);
  glVertex3f(x0,y0,z0);
  glVertex3f(x1,y1,z1);
  glEnd();

  if (lighting)
    glEnable(GL_LIGHTING);

}

