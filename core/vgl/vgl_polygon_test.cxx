/*
  fsm@robots.ox.ac.uk
*/
#ifdef __GNUC__
#pragma implementation "vgl_polygon_test"
#endif
#include "vgl_polygon_test.h"
#include <vcl/vcl_cmath.h>
#include <vgl/vgl_lineseg_test.h>

template <class T>
bool vgl_polygon_test_inside(T const *xs, T const *ys, unsigned n, T x, T y) 
{
  // compute centre
  T cx = 0;
  T cy = 0;
  for (unsigned i=0; i<n; ++i) {
    cx += xs[i];
    cy += ys[i];
  }
  cx /= n;
  cy /= n;

  // compute a point outside the polygon.
  T ox = 0, oy = 0;
  for (unsigned i=0; i<n; ++i) {
    T tmp;

    tmp = xs[i]-cx; 
    if (tmp<0) tmp = -tmp;
    if (tmp>ox) ox = tmp;
    
    tmp = ys[i]-cy; 
    if (tmp<0) tmp = -tmp;
    if (tmp>oy) oy = tmp;
  }
  ox = cx + ox + oy + 1;
  oy = cy + ox + oy + 1;
  
  // count crossings.
  unsigned crossings = 0;
  for (unsigned i=0; i<n; ++i)
    if (vgl_lineseg_test(xs[i], ys[i], xs[(i+1)%n], ys[(i+1)%n],   ox, oy, x, y))
      ++crossings;
  
  // inside iff there was an odd number of crossings.
  return crossings % 2;
}

//--------------------------------------------------------------------------------

#define inst(T) \
template bool vgl_polygon_test_inside(T const *, T const *, unsigned, T, T);
inst(float);
inst(double);
