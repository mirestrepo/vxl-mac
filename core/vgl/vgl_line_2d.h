#ifndef vgl_line_2d_h
#define  vgl_line_2d_h
#ifdef __GNUC__
#pragma interface
#endif

// Author: Don Hamilton, Peter Tu
// Copyright:
// Created: Feb 16 2000
// Modifications:
//  Peter Vanroose, 29 Feb 2000: several minor fixes

#include <vcl/vcl_iostream.h>
#include <vcl/vcl_cmath.h> // for sqrt()

template <class Type>
class vgl_point_2d;

template <class Type>
class vgl_homg_line_2d;

//: Represents a euclidian 2D line.
//   An interface for the line coefficients, [a,b,c], is provided
//   in terms of the standard implicit line equation:
//   a*x + b*y + c = 0
template <class Type>
class vgl_line_2d {

// PUBLIC INTERFACE--------------------------------------------------------
  
public:
 
  // Constructors/Initializers/Destructors-----------------------------------

  // Default constructor  
  // vgl_line_2d () {}
  
  // Default copy constructor  
  // vgl_line_2d (const vgl_line_2d<Type>& that) {
  //   data_[0]=that.data_[0];
  //   data_[1]=that.data_[1];
  //   data_[2]=that.data_[2];
  // }

  vgl_line_2d<Type> (vgl_homg_line_2d<Type> const& p);

  //: -- Construct a vgl_line_2d from its equation, three Types.
  vgl_line_2d (Type a, Type b, Type c) { data_[0]=a; data_[1]=b; data_[2]=c; }
  
  // -- Construct from two distinct points
  vgl_line_2d (vgl_point_2d<Type> const& p1,
               vgl_point_2d<Type> const& p2);
    
  //: -- Construct from its equation, a 3-vector.
  vgl_line_2d (const Type v[3]) { data_[0]=v[0];data_[1]=v[1];data_[2]=v[2]; }
  
  // Default destructor:
  // ~vgl_line_2d () {}
  
  // Default assignment operator:
  // vgl_line_2d<Type>& operator=(const vgl_line_2d<Type>& that){
  //   data_[0]=that.data_[0];
  //   data_[1]=that.data_[1];
  //   data_[2]=that.data_[2];
  //   return *this;
  // }

  // Data Access-------------------------------------------------------------

  inline void get_direction(Type& dx, Type& dy) const { dx=dir_x(); dy=dir_y(); }
  inline void get_normal(Type& nx, Type& ny) const { ny=normal_x(); ny=normal_y(); }

  inline Type dir_x() const { return -b()/(sqrt(a()*a() + b()*b())); }
  inline Type dir_y() const { return a()/(sqrt(a()*a() + b()*b())); }

  inline Type normal_x() const { return -dir_y(); }
  inline Type normal_y() const { return dir_x(); }

  inline Type a() const {return data_[0];}
  inline Type b() const {return data_[1];}
  inline Type c() const {return data_[2];}

  //: -- Set a b c.
  void set (Type a, Type b, Type c){ data_[0] = a; data_[1] = b; data_[2] = c; }

  //: find the distance of the line to the origin
  Type distance_to_origin() const;

  //: Get two points on the line; normally the intersection with X and Y axes.
  void get_two_points(vgl_point_2d<Type> &p1, vgl_point_2d<Type> &p2);
    
  
  // INTERNALS---------------------------------------------------------------

protected:
  // the data associated with this point 
  Type data_[3];
};

//: stream operators 
  
template <class Type>
ostream&  operator<<(ostream& s, const vgl_line_2d<Type>& p) {
  return s << " <vgl_line_2d " << p->data_[0] << " x + " << p->data_[1]
           << " y + " << p->data_[2] << " = 0>";
}

template <class Type>
istream&  operator>>(istream& is,  vgl_line_2d<Type>& p) {
  return is >> p->data_[0] >> p->data_[1] >> p->data_[2];
}
  
#endif
