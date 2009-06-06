// This is core/vgl/vgl_sphere_3d.h
#ifndef vgl_sphere_3d_h
#define vgl_sphere_3d_h
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief a sphere in 3D nonhomogeneous space
// \author Ian Scott

#include <vgl/vgl_fwd.h> // forward declare vgl_line_3d_2_points
#include <vgl/vgl_point_3d.h>

//: Represents a cartesian 3D point
template <class Type>
class vgl_sphere_3d
{
  vgl_point_3d<Type> c_; //!< centre
  Type r_;               //!< radius
 public:

  // Constructors/Initializers/Destructor------------------------------------

  //: Default constructor
   inline vgl_sphere_3d (): c_(0.0, 0.0, 0.0), r_(-1) {}

  //: Construct from four scalars: centre and radius.
  inline vgl_sphere_3d(Type px, Type py, Type pz, Type radius) : c_(px, py, pz), r_(radius) {}

  //: Construct from a 4-array, representing centre and radius.
  inline vgl_sphere_3d (const Type v[4]): c_(v[0], v[1], v[2]), r_(v[3]) {}

  //: Construct from centre point and radius.
  vgl_sphere_3d (vgl_point_3d<Type> const& centre, Type radius): c_(centre), r_(radius) {}

  //: Test for equality
  inline bool operator==(const vgl_sphere_3d<Type> &s) const
  { return this==&s || (c_==s.c_ && r_==s.r_); }
  inline bool operator!=(vgl_sphere_3d<Type>const& s)const
  { return !operator==(s); }

  // Data Access-------------------------------------------------------------

  inline const vgl_point_3d<Type> & centre() const {return c_;}
  inline Type radius() const {return r_;}

  //: Return true if this sphere is empty
  inline bool is_empty() const {
    return r_ < 0.0;
  }

  //: Return true iff the point p is inside (or on) this sphere
  bool contains(vgl_point_3d<Type> const& p) const;

  //: Make the sphere empty.
  void set_empty() {c_.set(0,0,0); r_=-1;}

  //: Set min \a x ordinate of box (other sides unchanged)
  inline void set_radius(Type r) { r_=r; }
  //: Set min \a y ordinate of box (other sides unchanged)
  inline void set_centre(const vgl_point_3d<Type> & c) { c_=c; }

  //: Calculate the end points of a line clipped by this sphere.
  bool clip(const vgl_line_3d_2_points<Type> & line,
            vgl_point_3d<Type> &p1, vgl_point_3d<Type> &p2) const;
};

#define VGL_SPHERE_3D_INSTANTIATE(T) extern "please include vgl/vgl_sphere_3d.txx first"

#endif // vgl_sphere_3d_h
