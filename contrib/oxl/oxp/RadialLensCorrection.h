#ifndef RadialLensCorrection_h_
#define RadialLensCorrection_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	RadialLensCorrection
// .LIBRARY	POX
// .HEADER	Oxford Package
// .INCLUDE	oxp/RadialLensCorrection.h
// .FILE	RadialLensCorrection.cxx
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 13 Jul 98
//
//-----------------------------------------------------------------------------

#include <vbl/vbl_printf.h>
#include <oxp/Mapping_2d_2d.h>

class RadialLensCorrection : public Mapping_2d_2d {
public:
  RadialLensCorrection(double cx, double cy, double sx, double sy, double k2, double k4 = 0);
  RadialLensCorrection(double k2);
  RadialLensCorrection(int w, int h, double k2);

protected: // implementation of Mapping_2d_2d
  virtual void implement_map(double x1, double y1, double* x2, double* y2);
  virtual void implement_inverse_map(double x2, double y2, double* x1, double* y1);

protected:
  void init(double cx, double cy, double sx, double sy, double k2, double k4);

protected:
  double _cx;
  double _cy;
  double _sx;
  double _sy;
  double _k2;
  double _k4;
  double _invsx;
  double _invsy;
};

#endif // RadialLensCorrection_h_
