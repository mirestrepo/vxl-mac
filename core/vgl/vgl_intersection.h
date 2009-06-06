// This is core/vgl/vgl_intersection.h
#ifndef vgl_intersection_h_
#define vgl_intersection_h_
//:
// \file
// \brief Set of intersection functions
// \author Jan 25, 2007 Gamze Tunali
//
// This file aims to gather all the intersection methods involving two or more types
// into one place. You will also find overloads of vgl_intersection in
// vgl/algo/vgl_intersection if they require significant computation,
// and in some ordinary files if they only involve one type
// (e.g. vgl_box2d& vgl_intersection(vgl_box_2d&, vgl_box_2d&) is in vgl_box_2d.h)
//
// For intersections of "homogeneous coordinates" objects like vgl_homg_line_2d<T>,
// see the static methods of vgl/algo/vgl_homg_operators_2d<T> and _3d.
//
// \verbatim
//  Modifications
//   01 Mar 2007 - Gamze Tunali - split up into vgl/algo and vgl parts
// \endverbatim

#include <vgl/vgl_fwd.h> // forward declare various vgl classes

//: Return true if line intersects box. If so, compute intersection points.
// \relates vgl_line_2d
template <class Type>
bool vgl_intersection(const vgl_box_2d<Type>& box,
                      const vgl_line_2d<Type>& line,
                      vgl_point_2d<Type>& p0,
                      vgl_point_2d<Type>& p1);

//: Returns the number of intersections of a line segment with a box, up to two are returned in p0 and p1.
// \relates vgl_line_segment_2d
template <class Type>
unsigned vgl_intersection(const vgl_box_2d<Type>& box,
                          const vgl_line_segment_2d<Type>& line,
                          vgl_point_2d<Type>& p0,
                          vgl_point_2d<Type>& p1);

//: Return the intersection point of two concurrent lines
// \relates vgl_line_3d_2_points
template <class T>
vgl_point_3d<T> vgl_intersection(vgl_line_3d_2_points<T> const& l1,
                                 vgl_line_3d_2_points<T> const& l2);

//: Return the intersection point of segments of two concurrent lines
// \relates vgl_line_segment_3d
template <class T>
bool vgl_intersection(vgl_line_segment_3d<T> const& l1,
                      vgl_line_segment_3d<T> const& l2,
                      vgl_point_3d<T>& i_pnt);

//: Return the intersection point of segments of a concurrent line and line segment pair.
// \relates vgl_line_segment_3d
// \relates vgl_line_3d_2_points
template <class T>
bool vgl_intersection(vgl_line_3d_2_points<T> const& l1,
                      vgl_line_segment_3d<T> const& l2,
                      vgl_point_3d<T>& i_pnt);

template <class T> inline
bool vgl_intersection(vgl_line_segment_3d<T> const& l1,
                      vgl_line_3d_2_points<T> const& l2,
                      vgl_point_3d<T>& i_pnt)
{
  return vgl_intersection(l2, l1, i_pnt);
}

//: Return the intersection point of two lines. Return false if lines are parallel
// \relates vgl_line_2d
template <class T>
bool vgl_intersection(const vgl_line_2d<T>  &line0,
                      const vgl_line_2d<T>  &line1,
                      vgl_point_2d<T>       &intersection_point );

//: Return the intersection point of a line and a plane.
// \relates vgl_line_3d_2_points
// \relates vgl_plane_3d
template <class T>
vgl_point_3d<T> vgl_intersection(vgl_line_3d_2_points<T> const& line,
                                 vgl_plane_3d<T> const& plane);

//: Return the intersection point of a line and a plane.
// \relates vgl_line_segment_3d
// \relates vgl_plane_3d
template <class T>
bool vgl_intersection(vgl_line_segment_3d<T> const& line,
                      vgl_plane_3d<T> const& plane,
                      vgl_point_3d<T> & i_pt);

//: Return the intersection point of three planes.
// \relates vgl_plane_3d
template <class T>
vgl_point_3d<T> vgl_intersection(const vgl_plane_3d<T>& p1,
                                 const vgl_plane_3d<T>& p2,
                                 const vgl_plane_3d<T>& p3);

//: Return true if any point on [p1,p2] is within tol of [q1,q2]
//  Tests two line segments for intersection or near intersection
//  (within given tolerance).
// \author Dan jackson
// \relates vgl_point_2d
template <class T>
bool vgl_intersection(vgl_point_2d<T> const& p1,
                      vgl_point_2d<T> const& p2,
                      vgl_point_2d<T> const& q1,
                      vgl_point_2d<T> const& q2,
                      double tol = 1e-6);

//: Return true if the box and polygon regions intersect, regions include boundaries
// \relates vgl_polygon
template <class T>
bool vgl_intersection(const vgl_box_2d<T>& b, const vgl_polygon<T>& poly);


#define VGL_INTERSECTION_INSTANTIATE(T) extern "please include vgl/vgl_intersection.txx first"
#define VGL_INTERSECTION_BOX_INSTANTIATE(T) extern "please include vgl/vgl_intersection.txx first"

#endif // vgl_intersection_h_
