#ifndef vgui_draw_line_h_
#define vgui_draw_line_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vgui_draw_line
// .INCLUDE vgui/internals/vgui_draw_line.h
// .FILE internals/vgui_draw_line.cxx
// @author fsm@robots.ox.ac.uk

//--------------------------------------------------------------------------------

// draw infinite line spanned in space by two points.

// from total projection matrix T = P * M.
bool vgui_draw_line(double const T[4][4], double const X[4], double const Y[4]);

// from projection and modelview matrices, P and M.
bool vgui_draw_line(double const P[4][4], double const M[4][4], double const X[4], double const Y[4]);

// easy.
bool vgui_draw_line(double const X[4], double const Y[4]);

//--------------------------------------------------------------------------------

// draw infinite line { ax+by+cw=0, z=0 }

// from total projection matrix T = P * M.
bool vgui_draw_line(double const T[4][4], double a, double b, double c);

// from projection and modelview matrices, P and M.
bool vgui_draw_line(double const P[4][4], double const M[4][4], double a, double b, double c);

// easy.
bool vgui_draw_line(double a, double b, double c);

#endif // vgui_draw_line_h_
