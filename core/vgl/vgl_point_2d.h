#ifndef vgl_point_2d_h
#define vgl_point_2d_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vgl/vgl_point_2d.h

//:
// \file
// \brief a point in 2D nonhomogeneous space
// \author Don Hamilton, Peter Tu
// \verbatim
// Modifications :
//  2 July 2001 Peter Vanroose implemented constructor from homg point
// 29 June 2001 Peter Vanroose moved arithmetic operators to new vgl_vector_2d
// \endverbatim

#include <vcl_iosfwd.h>
#include <vgl/vgl_fwd.h> // declare vgl_homg_point_2d and vgl_line_2d
#include <vgl/vgl_vector_2d.h>
#include <vcl_vector.h>

//: Represents a cartesian 2D point
template <class Type>
class vgl_point_2d {

  // PUBLIC INTERFACE--------------------------------------------------------

public:

  // Constructors/Initializers/Destructor------------------------------------

  //: Default constructor
  vgl_point_2d () {}

  //: Construct from two Types.
  vgl_point_2d (Type px, Type py) : x_(px), y_(py) {}

  //: Construct from 2-array.
  vgl_point_2d (Type const v[2]) : x_(v[0]), y_(v[1]) {}

  //: Construct from homogeneous point
  vgl_point_2d (vgl_homg_point_2d<Type> const& p);

  //: Construct from 2 lines (intersection).
  vgl_point_2d (vgl_line_2d<Type> const& l1,
                vgl_line_2d<Type> const& l2);

#if 0 // The compiler defaults for these are doing what they should do:
  //: Copy constructor
  vgl_point_2d(vgl_point_2d<Type> const& p) : x_(p.x()), y_(p.y()) {}
  //: Destructor
  ~vgl_point_2d () {}
#endif

  //: Assignment
  inline vgl_point_2d<Type>& operator=(const vgl_point_2d<Type>& that) {
    x_ = that.x(); y_ = that.y(); return *this;
  }

  //: Test for equality
  inline bool operator==(const vgl_point_2d<Type> &p) const {
    return this==&p || (x_==p.x() && y_==p.y());
  }
  inline bool operator!=(const vgl_point_2d<Type> &p) const { return !operator==(p); }

  // Data Access-------------------------------------------------------------

  inline Type x() const {return x_;}
  inline Type y() const {return y_;}

  //: Set x and y
  inline void set (Type px, Type py){ x_ = px; y_ = py; }
  //: Set x and y
  inline void set (Type const p[2]) { x_ = p[0]; y_ = p[1]; }
  inline void set_x (Type px) { x_ = px; }
  inline void set_y (Type py) { y_ = py; }

  // INTERNALS---------------------------------------------------------------

private:
  // the data associated with this point
  Type x_;
  Type y_;
};

//  +-+-+ point_2d simple I/O +-+-+

//: Write "<vgl_point_2d x,y>" to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, vgl_point_2d<Type> const& p);

//: Read x y from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& s, vgl_point_2d<Type>& p);

//  +-+-+ point_2d arithmetic +-+-+

//: Return true iff the point is at infinity (an ideal point).
// Always false.
template <class Type>
inline bool is_ideal(vgl_point_2d<Type> const&, Type) { return false; }

//: The difference of two points is the vector from second to first point
template <class Type>
inline vgl_vector_2d<Type> operator-(vgl_point_2d<Type> const& p1,
                                     vgl_point_2d<Type> const& p2) {
  return vgl_vector_2d<Type>(p1.x()-p2.x(), p1.y()-p2.y());
}

//: Adding a vector to a point gives a new point at the end of that vector
// Note that vector + point is not defined!  It's always point + vector.
template <class Type>
inline vgl_point_2d<Type> operator+(vgl_point_2d<Type> const& p,
                                    vgl_vector_2d<Type> const& v) {
  return vgl_point_2d<Type>(p.x()+v.x(), p.y()+v.y());
}

//: Adding a vector to a point gives the point at the end of that vector
template <class Type>
inline vgl_point_2d<Type>& operator+=(vgl_point_2d<Type>& p,
                                      vgl_vector_2d<Type> const& v) {
  p.set(p.x()+v.x(), p.y()+v.y()); return p;
}

//: Subtracting a vector from a point is the same as adding the inverse vector
template <class Type>
inline vgl_point_2d<Type> operator-(vgl_point_2d<Type> const& p,
                                    vgl_vector_2d<Type> const& v) {
  return p + (-v);
}

//: Subtracting a vector from a point is the same as adding the inverse vector
template <class Type>
inline vgl_point_2d<Type>& operator-=(vgl_point_2d<Type>& p,
                                      vgl_vector_2d<Type> const& v) {
  return p += (-v);
}

//  +-+-+ point_2d geometry +-+-+

//: Are three points collinear, i.e., do they lie on a common line?
template <class Type>
inline bool collinear(vgl_point_2d<Type> const& p1,
                      vgl_point_2d<Type> const& p2,
                      vgl_point_2d<Type> const& p3) {
  return parallel(p1-p2, p1-p3);
}

//: Return the relative distance to p1 wrt p1-p2 of p3.
//  The three points should be collinear and p2 should not equal p1.
//  This is the coordinate of p3 in the affine 1D reference frame (p1,p2).
//  If p3=p1, the ratio is 0; if p1=p3, the ratio is 1.
//  The mid point of p1 and p2 has ratio 0.5.
//  Note that the return type is double, not Type, since the ratio of e.g.
//  two vgl_vector_2d<int> need not be an int.
template <class Type>
inline double ratio(vgl_point_2d<Type> const& p1,
                    vgl_point_2d<Type> const& p2,
                    vgl_point_2d<Type> const& p3) {
  return (p3-p1)/(p2-p1);
}

//: Return the point at a given ratio wrt two other points.
//  By default, the mid point (ratio=0.5) is returned.
//  Note that the third argument is Type, not double, so the midpoint of e.g.
//  two vgl_point_2d<int> is not a valid concept.  But the reflection point
//  of p2 wrt p1 is: in that case f=-1.
template <class Type>
inline vgl_point_2d<Type> midpoint(vgl_point_2d<Type> const& p1,
                                   vgl_point_2d<Type> const& p2,
                                   Type f = 0.5) {
  return p1 + f*(p2-p1);
}


//: Return the point at the centre of gravity of two given points.
// Identical to midpoint(p1,p2).
template <class Type>
inline vgl_point_2d<Type> centre(vgl_point_2d<Type> const& p1,
                                 vgl_point_2d<Type> const& p2) {
  return vgl_point_2d<Type>((p1.x() + p2.x())/2 ,
                            (p1.y() + p2.y())/2 );
}

//: Return the point at the centre of gravity of three given points.
template <class Type>
inline vgl_point_2d<Type> centre(vgl_point_2d<Type> const& p1,
                                 vgl_point_2d<Type> const& p2,
                                 vgl_point_2d<Type> const& p3) {
  return vgl_point_2d<Type>((p1.x() + p2.x() + p3.x())/3 ,
                            (p1.y() + p2.y() + p3.y())/3 );
}

//: Return the point at the centre of gravity of four given points.
template <class Type>
inline vgl_point_2d<Type> centre(vgl_point_2d<Type> const& p1,
                                 vgl_point_2d<Type> const& p2,
                                 vgl_point_2d<Type> const& p3,
                                 vgl_point_2d<Type> const& p4) {
  return vgl_point_2d<Type>((p1.x() + p2.x() + p3.x() + p4.x())/4 ,
                            (p1.y() + p2.y() + p3.y() + p4.y())/4 );
}

//: Return the point at the centre of gravity of a set of given points.
// Beware of possible rounding errors when Type is e.g. int.
template <class Type>
inline vgl_point_2d<Type> centre(vcl_vector<vgl_point_2d<Type> > const& v) {
  int n=v.size();
  assert(n>0); // it is *not* correct to return the point (0,0) when n==0.
  Type x = 0, y = 0;
  for (int i=0; i<n; ++i) x+=v[i].x(), y+=v[i].y();
  return vgl_point_2d<Type>(x/n,y/n);
}

#define VGL_POINT_2D_INSTANTIATE(T) extern "please include vgl/vgl_point_2d.txx first"

#endif // vgl_point_2d_h
