// This is oxl/vgui/vgui_section_render.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
// \file
// \author fsm

#include "vgui_section_render.h"

#include <vcl_cmath.h>
#include <vcl_cassert.h>

#include <vgui/internals/vgui_rasterpos.h>
#include <vgui/internals/vgui_accelerate.h>

static inline float fsm_max(float x, float y) { return x>y ? x : y; }
static inline float fsm_min(float x, float y) { return x<y ? x : y; }

// Set to 1 for verbose debugging.
#if 0
# include <stdio.h>
# define fsm_debug printf
#else
static inline void fsm_debug(char const *, ...) { }
#endif

//
bool vgui_section_render(void const *pixels,
                         unsigned w, unsigned h,
                         float x0, float y0,
                         float x1, float y1,
                         GLenum format,
                         GLenum type )
{
  assert(pixels);
  assert(x0 <= x1);
  assert(y0 <= y1);

  // Get current matrix state, in fortran order.
  double Pt[4][4], Mt[4][4];
  glGetDoublev(GL_PROJECTION_MATRIX, &Pt[0][0]);
  glGetDoublev(GL_MODELVIEW_MATRIX,  &Mt[0][0]);

  // Get total world-to-device transformation. It should be of the form :
  // * 0 0 *
  // 0 * 0 *
  // 0 0 * *
  // 0 0 0 *
  // with the diagonal entries non-zero. If not of this form, return false.
  double T[4][4];
  for (unsigned i=0; i<4; ++i) {
    for (unsigned j=0; j<4; ++j) {
      T[i][j] = 0;
      for (unsigned k=0; k<4; ++k)
        T[i][j] += Pt[k][i] * Mt[j][k]; // Pt[k][i] = P[i][k] etc
    }
  }
  if (!T[0][0]|| T[0][1] || T[0][2] ||
      T[1][0] ||!T[1][1] || T[1][2] ||
      T[2][0] || T[2][1] ||!T[2][2] ||
      T[3][0] || T[3][1] || T[3][2] || !T[3][3])
    return false; // cannot do

  // From image to device coordinates, the projection is :
  // [ T00  0  T03 ]   [ a   u ]
  // [  0  T11 T13 ] ~ [   b v ]
  // [  0   0  T33 ]   [     1 ]
  float a = T[0][0]/T[3][3], b = T[1][1]/T[3][3];
  float u = T[0][3]/T[3][3], v = T[1][3]/T[3][3];

  // Get size of viewport. We need this to determine how much to scale pixels by.
  GLint vp[4]; // x,y, w,h
  glGetIntegerv(GL_VIEWPORT, vp);
  //int vp_x = vp[0];
  //int vp_y = vp[1];
  int vp_w = vp[2];
  int vp_h = vp[3];
  if (vp_w <= 0 || vp_h <= 0)
    return true;


  // From device to viewport coordinates, the transformation is :
  // [ vp_w   0  vp_x ] [ 1/2  0  1/2 ]
  // [   0  vp_h vp_y ] [  0  1/2 1/2 ]
  // [   0    0    1  ] [  0   0   1  ]
  // where vp_x, vp_y, vp_w, vp_h are the start, width and height of the viewport.

  // Compute pixel zoom, as passed to glPixelZoom().
  float zoomx = a*vp_w/2;
  float zoomy = b*vp_h/2;

  // Clip the given region [x0, x1] x [y0, y1] to the viewport.  In
  // device coordinates, the viewport is [-1, +1] x [-1, +1] so it's
  // easiest to start from that. This clipping is especially important
  // for non-local displays, where the display clipping happens on the
  // display server.
  //
  if (a>0) {
    // [ (-1-u)/a, (+1-u)/a ]
    x0 = fsm_max(x0, (-1-u)/a);
    x1 = fsm_min(x1, (+1-u)/a);
  }
  else {
    // [ (+1-u)/a, (-1-u)/a ]
    x0 = fsm_max(x0, (+1-u)/a);
    x1 = fsm_min(x1, (-1-u)/a);
  }
  if (b>0) {
    // [ (-1-v)/b, (+1-v)/b ]
    y0 = fsm_max(y0, (-1-v)/b);
    y1 = fsm_min(y1, (+1-v)/b);
  }
  else {
    // [ (+1-v)/b, (-1-v)/b ]
    y0 = fsm_max(y0, (+1-v)/b);
    y1 = fsm_min(y1, (-1-v)/b);
  }
  if (x0 > x1 || y0 > y1) {
    fsm_debug("nothing to render\n");
    return true; // that's easy.
  }

  // Before dumping the image, we have to set a valid raster
  // position. However, to get a smooth panning effect, we want to
  // render the (potentially) partial pixels at the borders, which
  // means we must get the raster position to an "invalid" position
  // outside the viewport. vgui_rasterpos wraps the appropriate
  // trickery.

  int i_x0 = int(vcl_floor(x0)), i_y0 = int(vcl_floor(y0));
  int i_x1 = int(vcl_ceil (x1)), i_y1 = int(vcl_ceil (y1));

  vgui_rasterpos2i( i_x0, i_y0 );

  // this should never happen, but it doesn't hurt to fail gracefully.
  if (!vgui_rasterpos_valid()) {
    fsm_debug("invalid rasterpos\n");
    return false; //
  }

  // Store old transfer characteristics for restoring it in a bit.
  GLint alignment, row_length, skip_pixels, skip_rows;
  glGetIntegerv(GL_UNPACK_ALIGNMENT,   &alignment);
  glGetIntegerv(GL_UNPACK_ROW_LENGTH,  &row_length);
  glGetIntegerv(GL_UNPACK_SKIP_PIXELS, &skip_pixels);
  glGetIntegerv(GL_UNPACK_SKIP_ROWS,   &skip_rows);
  

  // Set pixel transfer characteristics.
  glPixelStorei(GL_UNPACK_ALIGNMENT,   1);         // use byte alignment for now.
  glPixelStorei(GL_UNPACK_ROW_LENGTH,  w);         // size of image rows.
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, i_x0);      // number of pixels to skip on the left.
  glPixelStorei(GL_UNPACK_SKIP_ROWS,   i_y0);      // number of pixels to skip at the bottom.

  glPixelZoom( zoomx, zoomy );
  vgui_accelerate::instance()->vgui_glDrawPixels(i_x1 - i_x0, // Size of pixel rectangle
                                                 i_y1 - i_y0, // to be written to frame buffer.
                                                 format,
                                                 type,
                                                 pixels);

  // Restore previous values.
  glPixelStorei(GL_UNPACK_ALIGNMENT,   alignment);
  glPixelStorei(GL_UNPACK_ROW_LENGTH,  row_length);
  glPixelStorei(GL_UNPACK_SKIP_ROWS  , skip_pixels);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, skip_rows);

  return true; // could do
}

//--------------------------------------------------------------------------------
