#ifndef vgl_homg_line_2d_h
#define  vgl_homg_line_2d_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vgl/vgl_homg_line_2d.h

//:
// \file
// \brief line in projective 2D space
// \author Don Hamilton, Peter Tu
//
// \verbatim
// Modifications
// Peter Vanroose -  6 July 2001 - Added normal(), direction() and concurrent()
// Peter Vanroose -  4 July 2001 - Added assertions and cstr from non-homg line
// Peter Vanroose - 27 June 2001 - Added operator==
// \endverbatim

#include <vcl_iosfwd.h>
#include <vgl/vgl_fwd.h> // forward declare vgl_homg_point_2d and vgl_line_2d
#include <vcl_cassert.h>

//: Represents a homogeneous 2D line.
template <class Type>
class vgl_homg_line_2d {

  // PUBLIC INTERFACE--------------------------------------------------------

public:

  // Constructors/Initializers/Destructor------------------------------------

  //: Default constructor (Line 1.y==0, the X axis)
  vgl_homg_line_2d () : a_(0), b_(1), c_(0) {}

  // Default copy constructor
  vgl_homg_line_2d (const vgl_homg_line_2d<Type>& l) : a_(l.a()), b_(l.c()), c_(l.c()) {}

  //: Construct from three Types.
  //  The three given numbers should not be all 0
  vgl_homg_line_2d (Type a, Type b, Type c) : a_(a), b_(b), c_(c) {assert(a||b||c);}

  //: Construct from 3-vector.
  //  The three given numbers should not be all 0
  vgl_homg_line_2d (const Type v[3]) : a_(v[0]), b_(v[1]), c_(v[2]) {assert(a_||b_||c_);}

  //: Construct from non-homogeneous line
  vgl_homg_line_2d<Type> (vgl_line_2d<Type> const& p);

  //: Construct from two distinct points (join)
  //  The two points must be distinct!
  vgl_homg_line_2d (vgl_homg_point_2d<Type> const& p1, vgl_homg_point_2d<Type> const& p2);

#if 0
  // Default destructor
  ~vgl_homg_line_2d () {}

  // Default assignment operator
  vgl_homg_line_2d<Type>& operator=(const vgl_homg_line_2d<Type>& that){
    set(that.a(),that.b(),that.c()); return *this;
  }
#endif

  //: the equality operator
  bool operator==(vgl_homg_line_2d<Type> const& other) const;
  bool operator!=(vgl_homg_line_2d<Type> const& other) const { return ! operator==(other); }

  // Data Access-------------------------------------------------------------

  //: Parameter a of line a*x + b*y + c*w = 0
  inline Type a() const {return a_;}
  //: Parameter b of line a*x + b*y + c*w = 0
  inline Type b() const {return b_;}
  //: Parameter c of line a*x + b*y + c*w = 0
  inline Type c() const {return c_;}

  //: unit vector describing line direction, or (0,0) if line at infinity
  inline vgl_vector_2d<double> direction() const { return normalized(vgl_vector_2d<double>(b_,-a_)); }

  //: unit vector orthogonal to line, or (0,0) if line at infinity
  inline vgl_vector_2d<double> normal() const { return normalized(vgl_vector_2d<double>(a_,b_)); }

protected: // \deprecated
  //: x component of unit vector describing direction of line
  inline double dir_x() const { return direction().x(); }

  //: y component of unit vector describing direction of line
  inline double dir_y() const { return direction().y(); }

  //: x component of unit vector orthogonal to line
  inline double normal_x() const { return normal().x(); }

  //: y component of unit vector orthogonal to line
  inline double normal_y() const { return normal().y(); }

public:
  //: Set a b c.
  //  The three given numbers should not be all 0
  //  Note that it does not make sense to set a, b or c separately
  inline void set (Type a, Type b, Type c) {assert(a||b||c); a_=a; b_=b; c_=c;}

  //: Return true iff this line is the line at infinity
  //  This version checks (max(|a|,|b|) <= tol * |c|
  bool ideal(Type tol = Type(0)) const {
#define vgl_Abs(x) (x<0?-x:x) // avoid #include of vcl_cmath.h AND vcl_cstdlib.h
    return vgl_Abs(a()) <= tol*vgl_Abs(c()) && vgl_Abs(b()) <= tol*vgl_Abs(c());
#undef vgl_Abs
  }

  //:get two points on the line
  // These two points are normally the intersections
  // with the Y axis and X axis, respectively.  When the line is parallel to one
  // of these, the point with y=1 or x=1, resp. are taken.  When the line goes
  // through the origin, the second point is (b, -a, 1).  Finally, when the line
  // is the line at infinity, the returned points are (1,0,0) and (0,1,0).
  // Thus, whenever possible, the returned points are not at infinity.
  void get_two_points(vgl_homg_point_2d<Type> &p1, vgl_homg_point_2d<Type> &p2);

  // INTERNALS---------------------------------------------------------------

private:
  //: the data associated with this line
  Type a_;
  Type b_;
  Type c_;
};

#define l vgl_homg_line_2d<Type>

//: Return true iff line is the line at infinity
template <class Type>
bool is_ideal(l const& line, Type tol = Type(0)) { return line.ideal(tol); }

//: Are three lines concurrent, i.e., do they pass through a common point?
template <class Type>
inline bool concurrent(l const& l1, l const& l2, l const& l3) {
  return l1.a()*(l2.b()*l3.c()-l3.b()*l2.c())
        +l2.a()*(l3.b()*l1.c()-l1.b()*l3.c())
        +l3.a()*(l1.b()*l2.c()-l2.b()*l1.c())==0;
}

//: Print line equation to stream
template <class Type>
vcl_ostream& operator<<(vcl_ostream& s, l const& line);

//: Load in line parameters from stream
template <class Type>
vcl_istream& operator>>(vcl_istream& s, l& line);

#undef l

#define VGL_HOMG_LINE_2D_INSTANTIATE(T) extern "please include vgl/vgl_homg_line_2d.txx first"

#endif //  vgl_homg_line_2d_h
