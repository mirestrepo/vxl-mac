#ifndef vgl_point_3d_h
#define vgl_point_3d_h
#ifdef __GNUC__
#pragma interface
#endif

// This is vxl/vgl/vgl_point_3d.h

//:
// \file
// \brief a point in 3D nonhomogeneous space
// \author Don Hamilton, Peter Tu
//
// \verbatim
// Modifications
// Peter Vanroose -  2 July 2001 - Added constructor from 3 planes
// \endverbatim

#include <vcl_iosfwd.h>
#include <vgl/vgl_fwd.h> // forward declare vgl_plane_3d
#include <vgl/vgl_vector_3d.h>

//: Represents a cartesian 3D point
template <class Type>
class vgl_point_3d {

  // PUBLIC INTERFACE--------------------------------------------------------

public:

  // Constructors/Initializers/Destructors-----------------------------------

  //: Default constructor
  vgl_point_3d () {}

  //: Copy constructor
  vgl_point_3d(const vgl_point_3d<Type>& p) : x_(p.x()), y_(p.y()), z_(p.z()) {}

  //: Construct from three Types.
  vgl_point_3d(Type px, Type py, Type pz) : x_(px), y_(py), z_(pz) {}

  //: Construct from 3-vector.
  vgl_point_3d (const Type v[3]) : x_(v[0]), y_(v[1]), z_(v[2]) {}

  //: Construct from 3 planes
  vgl_point_3d (const vgl_plane_3d<Type>& pl1,
                const vgl_plane_3d<Type>& pl2,
                const vgl_plane_3d<Type>& pl3); /* TODO */

  //: Construct from homogeneous point
  vgl_point_3d (vgl_homg_point_3d<Type> const& p);

  //: Destructor
  ~vgl_point_3d () {}

  //: Assignment
  vgl_point_3d<Type>& operator=(const vgl_point_3d<Type>& that){
    x_ = that.x(); y_ = that.y(); z_ = that.z();
    return *this;
  }

  //: Test for equality
  bool operator==(const vgl_point_3d<Type> &p) const {
    return this==&p || (x_==p.x() && y_==p.y() && z_==p.z());
  }
  bool operator!=(const vgl_point_3d<Type> &p) const { return !operator==(p); }

  // Data Access-------------------------------------------------------------

  inline Type x() const {return x_;}
  inline Type y() const {return y_;}
  inline Type z() const {return z_;}

  //: Set x,y and z
  inline void set (Type px, Type py, Type pz){ x_ = px; y_ = py; z_ = pz; }
  inline void set_x (Type px) { x_ = px; }
  inline void set_y (Type py) { y_ = py; }
  inline void set_z (Type pz) { z_ = pz; }

  //: Write "x y z " to stream s
  vcl_ostream& write(vcl_ostream& s) const;

  // INTERNALS---------------------------------------------------------------

private:
  // the data associated with this point

  Type x_;
  Type y_;
  Type z_;
};

//  +-+-+ point_3d simple I/O +-+-+

//: Write "<vgl_point_3d x,y,z> " to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, const vgl_point_3d<Type>& p);

//: Read x y z from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  vgl_point_3d<Type>& p);

//  +-+-+ point_3d arithmetic +-+-+

//: The difference of two points is the vector from second to first point
template <class Type>
inline vgl_vector_3d<Type> operator-(vgl_point_3d<Type> const& p1,
                                     vgl_point_3d<Type> const& p2) {
  return vgl_vector_3d<Type>(p1.x()-p2.x(), p1.y()-p2.y(), p1.z()-p2.z());
}

//: Adding a vector to a point gives a new point at the end of that vector
// Note that vector + point is not defined!  It's always point + vector.
template <class Type>
inline vgl_point_3d<Type> operator+(vgl_point_3d<Type> const& p,
                                    vgl_vector_3d<Type> const& v) {
  return vgl_point_3d<Type>(p.x()+v.x(), p.y()+v.y(), p.z()+v.z());
}

//: Subtracting a vector from a point is the same as adding the inverse vector
template <class Type>
inline vgl_point_3d<Type> operator-(vgl_point_3d<Type> const& p,
                                    vgl_vector_3d<Type> const& v) {
  return p + (-v);
}

//  +-+-+ point_3d geometry +-+-+

//: Are three points collinear, i.e., do they lie on a common line?
template <class Type>
inline bool collinear(vgl_point_3d<Type> const& p1,
                      vgl_point_3d<Type> const& p2,
                      vgl_point_3d<Type> const& p3) {
  return parallel(p1-p2, p1-p3);
}

//: Return the relative distance to p1 wrt p1-p2 of p3.
//  The three points should be collinear and p2 should not equal p1.
//  This is the coordinate of p3 in the affine 1D reference frame (p1,p2).
//  If p3=p1, the ratio is 0; if p1=p3, the ratio is 1.
//  The mid point of p1 and p2 has ratio 0.5.
//  Note that the return type is double, not Type, since the ratio of e.g.
//  two vgl_vector_3d<int> need not be an int.
template <class Type>
inline double ratio(vgl_point_3d<Type> const& p1,
                    vgl_point_3d<Type> const& p2,
                    vgl_point_3d<Type> const& p3) {
  return (p3-p1)/(p2-p1);
}

//: Return the point at a given ratio wrt two other points.
//  By default, the mid point (ratio=0.5) is returned.
//  Note that the third argument is Type, not double, so the midpoint of e.g.
//  two vgl_point_3d<int> is not a valid concept.  But the reflection point
//  of p2 wrt p1 is: in that case f=-1.
template <class Type>
inline vgl_point_3d<Type> midpoint(vgl_point_3d<Type> const& p1,
                                   vgl_point_3d<Type> const& p2,
                                   Type f = 0.5) {
  return p1 + f*(p2-p1);
}

#endif // vgl_point_3d_h
