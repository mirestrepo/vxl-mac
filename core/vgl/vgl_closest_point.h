// This is core/vgl/vgl_closest_point.h
#ifndef vgl_closest_point_h_
#define vgl_closest_point_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief Set of closest-point functions
// \author Peter Vanroose
//
// All these functions have two arguments which are geometry objects, and
// return either a pair of points (one from each object) which are the ones
// closest to each other, or just 1 point in case one of the objects is a point.
//
// See also vgl_distance if you only need the shortest distance between two
// geometric objects and not the actual closest points.
//
// \verbatim
//  Modifications
//   5 June 2003 Peter Vanroose created from bits and pieces in vgl_distance
//   5 June 2003 Brendan McCane added closest-point algo for 3D lines
// \endverbatim

#include <vgl/vgl_fwd.h> // forward declare various vgl classes
#include <vcl_utility.h> // for vcl_pair<T,U>

//: Closest point to \a (x,y) on the line segment \a (x1,y1)-(x2,y2)
void vgl_closest_point_to_linesegment(double& ret_x, double& ret_y,
                                      double x1, double y1,
                                      double x2, double y2,
                                      double x, double y);

//: Closest point to \a (x,y) on open polygon \a (px[i],py[i])
void vgl_closest_point_to_non_closed_polygon(double& ret_x, double& ret_y,
                                             float const px[], float const py[], unsigned n,
                                             double x, double y);

//: Closest point to \a (x,y) on closed polygon \a (px[i],py[i])
void vgl_closest_point_to_closed_polygon(double& ret_x, double& ret_y,
                                         float const px[], float const py[], unsigned n,
                                         double x, double y);

//: Closest point to the origin on the given line
// \relates vgl_line_2d
// \relates vgl_point_2d
template <class Type>
vgl_point_2d<Type> vgl_closest_point_origin(vgl_line_2d<Type> const& l);

//: Closest point to the origin on the given line
// \relates vgl_homg_line_2d
// \relates vgl_homg_point_2d
template <class Type>
vgl_homg_point_2d<Type> vgl_closest_point_origin(vgl_homg_line_2d<Type> const& l);

//: Closest point to the given point on the given line
// \relates vgl_point_2d
// \relates vgl_line_2d
template <class Type>
vgl_point_2d<Type> vgl_closest_point(vgl_line_2d<Type> const& l,
                                     vgl_point_2d<Type> const& p);
template <class Type> inline
vgl_point_2d<Type> vgl_closest_point(vgl_point_2d<Type> const& p,
                                     vgl_line_2d<Type> const& l) { return vgl_closest_point(l,p); }

//: Closest point to the given point on the given line
// \relates vgl_homg_point_2d
// \relates vgl_homg_line_2d
template <class Type>
vgl_homg_point_2d<Type> vgl_closest_point(vgl_homg_line_2d<Type> const& l,
                                          vgl_homg_point_2d<Type> const& p);
template <class Type> inline
vgl_homg_point_2d<Type> vgl_closest_point(vgl_homg_point_2d<Type> const& p,
                                          vgl_homg_line_2d<Type> const& l) { return vgl_closest_point(l,p); }

//: Closest point to the given point on the given plane
// \relates vgl_point_3d
// \relates vgl_plane_3d
template <class Type>
vgl_point_3d<Type> vgl_closest_point(vgl_plane_3d<Type> const& l,
                                     vgl_point_3d<Type> const& p);
template <class Type> inline
vgl_point_3d<Type> vgl_closest_point(vgl_point_3d<Type> const& p,
                                     vgl_plane_3d<Type> const& l) { return vgl_closest_point(l,p); }

//: Closest point to the given point on the given plane
// \relates vgl_homg_point_3d
// \relates vgl_homg_plane_3d
template <class Type>
vgl_homg_point_3d<Type> vgl_closest_point(vgl_homg_plane_3d<Type> const& l,
                                          vgl_homg_point_3d<Type> const& p);
template <class Type> inline
vgl_homg_point_3d<Type> vgl_closest_point(vgl_homg_point_3d<Type> const& p,
                                          vgl_homg_plane_3d<Type> const& l) { return vgl_closest_point(l,p); }

//: Closest point to the given point on the given polygon
//  If the third argument is "false", the edge from last to first point of
//  each polygon sheet is not considered part of the polygon.
// \relates vgl_point_2d
// \relates vgl_polygon
template <class Type>
vgl_point_2d<Type> vgl_closest_point(vgl_polygon const& poly,
                                     vgl_point_2d<Type> const& point,
                                     bool closed=true);

template <class Type> inline
vgl_point_2d<Type> vgl_closest_point(vgl_point_2d<Type> const& point,
                                     vgl_polygon const& poly,
                                     bool closed=true) { return vgl_closest_point(poly, point, closed); }

//: Return the two closest points between two lines in 3D.
// There are 3 cases: the lines intersect (hence these two points are equal);
// the lines are parallel (an infinite number of solutions viz all points);
// the lines are neither parallel nor do they intersect (the general case).
// This method handles all 3 cases. In all cases, a pair of points is returned;
// in case 2, these are both the common point at infinity of the two lines.
//
// \param const vgl_homg_line_3d_2_points<Type> &line1
// \param const vgl_homg_line_3d_2_points<Type> &line2
//
// \return vcl_pair<vgl_homg_point_3d<Type>, vgl_homg_point_3d<Type> > 
//
// \author Paul Bourke, modified for use in VXL by Brendan McCane
//
// \notes This routine is adapted from code written by Paul Bourke and
// available online at
// http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
//
// \notes I'm not sure if case 2 is mathematically correct. Especially
// if the two input lines are in fact the same line. Two parallel
// lines do intersect at infinity, but a line intersects itself
// everywhere, so I don't know what the correct mathematical answer to
// that one is.

template <class Type>
vcl_pair<vgl_homg_point_3d<Type>, vgl_homg_point_3d<Type> >
vgl_closest_points(const vgl_homg_line_3d_2_points<Type> &line1,
                   const vgl_homg_line_3d_2_points<Type> &line2);

#endif // vgl_closest_point_h_
