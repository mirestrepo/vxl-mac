// This is vxl/vgl/vgl_line_segment_2d.h
#ifndef vgl_line_segment_2d_h_
#define vgl_line_segment_2d_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author mccane@cs.otago.ac.nz: but copied from vgl_line_segment_3d
//
// \verbatim
// Modifications
// Peter Vanroose -  9 July 2001 - Inlined constructors
// Peter Vanroose - 27 June 2001 - Added operator==
// J.L. Mundy     - 13 April 2003 - Added angle and line coefficient functions
// \endverbatim

#include <vcl_iosfwd.h>
#include <vgl/vgl_point_2d.h> // data member of this class

//: Represents a 2D line segment using two points.
template <class Type>
class vgl_line_segment_2d
{
  //: One end of line segment
  vgl_point_2d<Type> point1_;
  //: The other end of the line segment
  vgl_point_2d<Type> point2_;

 public:
  //: Default constructor - does not initialise!
  inline vgl_line_segment_2d() {}

  //: Copy constructor
  inline vgl_line_segment_2d(vgl_line_segment_2d<Type> const& l)
    : point1_(l.point1_), point2_(l.point2_) {}

  //: Construct from two end points
  inline vgl_line_segment_2d(vgl_point_2d<Type> const& p1,
                             vgl_point_2d<Type> const& p2)
    : point1_(p1), point2_(p2) {}

  //: Destructor
  inline ~vgl_line_segment_2d() {}

  //: One end-point of the line segment.
  inline vgl_point_2d<Type> point1() const { return point1_; } // return a copy

  //: The other end-point of the line segment.
  inline vgl_point_2d<Type> point2() const { return point2_; } // return a copy

  //: The equality comparison operator
  inline bool operator==(vgl_line_segment_2d<Type> const& l) const {
    return (this==&l) || (point1() == l.point1() && point2() == l.point2())
                      || (point1() == l.point2() && point2() == l.point1()); }

  //: The inequality comparison operator.
  inline bool operator!=(vgl_line_segment_2d<Type>const& other)const{return !operator==(other);}

  //: A consistent interface with vgl_line_2d
  //: Parameter a of line a*x + b*y + c = 0
  Type a() const;

  //: Parameter b of line a*x + b*y + c = 0
  Type b() const;

  //: Parameter c of line a*x + b*y + c = 0
  Type c() const;

  //: unit vector describing line direction
  vgl_vector_2d<double> direction() const;

  //: unit vector orthogonal to line
  vgl_vector_2d<double> normal() const;

  //: tangent angle in degrees
  double tangent_angle() const;
  
  //: Assignment
  inline void set(vgl_point_2d<Type> const& p1, vgl_point_2d<Type> const& p2) {
    point1_ = p1; point2_ = p2; }
};

//: Write to stream
// \relates vgl_line_segment_2d
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, const vgl_line_segment_2d<Type>& p);

//: Read from stream
// \relates vgl_line_segment_2d
template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  vgl_line_segment_2d<Type>& p);

#define VGL_LINE_SEGMENT_2D_INSTANTIATE(T) extern "please include vgl/vgl_line_segment_2d.txx first"

#endif // vgl_line_segment_2d_h_
