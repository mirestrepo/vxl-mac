/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation
#endif
#include "vgui_rasterpos.h"
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vgui/vgui_matrix_state.h>

// Purpose : a RasterPos() routine which sets the raster position, even if GL thinks it is invalid.
void vgui_rasterpos4dv(double const X[4]) {
  GLint vp[4]; // x,y, w,h
  glGetIntegerv(GL_VIEWPORT, vp);
  
  vnl_matrix<double> T(vgui_matrix_state::total_transformation());
  vnl_vector<double> tmp = T * vnl_vector<double>(X, 4);
  float rx = tmp[0]/tmp[3];
  float ry = tmp[1]/tmp[3];
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  //           To set a valid raster position outside the viewport, first
  //          set a valid raster position inside the viewport, then call
  //          glBitmap with NULL as the bitmap parameter and with xmove
  //          and ymove set to the offsets of the new raster position.
  //          This technique is useful when panning an image around the
  //          viewport.
  // (from http://trant.sgi.com/opengl/docs/man_pages/hardcopy/GL/html/gl/bitmap.html)
  glRasterPos2f(0,0);
  glBitmap(0,0,
	   0,0,
	   rx*vp[2]/2, ry*vp[3]/2,
	   0);
}

void vgui_rasterpos2f(float x, float y) {
  double X[4]={x,y,0,1};
  vgui_rasterpos4dv(X);
}

bool vgui_rasterpos_valid() {
  GLboolean params;
  glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, &params);
  return (bool)params;
}
