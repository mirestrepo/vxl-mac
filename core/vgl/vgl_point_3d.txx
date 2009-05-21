// This is core/vgl/vgl_point_3d.txx
#ifndef vgl_point_3d_txx_
#define vgl_point_3d_txx_
//:
// \file

#include "vgl_point_3d.h"
#include <vgl/vgl_homg_point_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_homg_plane_3d.h>

#include <vcl_iostream.h>
#include <vcl_iomanip.h>

//: Construct from homogeneous point
template <class Type>
vgl_point_3d<Type>::vgl_point_3d(vgl_homg_point_3d<Type> const& p)
  : x_(p.x()/p.w()), y_(p.y()/p.w()), z_(p.z()/p.w()) // could be infinite!
{
}

//: Construct from 3 planes (intersection).
template <class Type>
vgl_point_3d<Type>::vgl_point_3d(vgl_plane_3d<Type> const& pl1,
                                 vgl_plane_3d<Type> const& pl2,
                                 vgl_plane_3d<Type> const& pl3)
{
  vgl_homg_plane_3d<Type> h1(pl1.nx(), pl1.ny(), pl1.nz(), pl1.d());
  vgl_homg_plane_3d<Type> h2(pl2.nx(), pl2.ny(), pl2.nz(), pl2.d());
  vgl_homg_plane_3d<Type> h3(pl3.nx(), pl3.ny(), pl3.nz(), pl3.d());
  vgl_homg_point_3d<Type> p(h1, h2, h3); // do homogeneous intersection
  set(p.x()/p.w(), p.y()/p.w(), p.z()/p.w()); // could be infinite!
}

template <class T>
double cross_ratio(vgl_point_3d<T>const& p1, vgl_point_3d<T>const& p2,
                   vgl_point_3d<T>const& p3, vgl_point_3d<T>const& p4)
{
  // least squares solution: (Num_x-CR*Den_x)^2 + (Num_y-CR*Den_y)^2 + (Num_z-CR*Den_z)^2 minimal.
  double Num_x = (p1.x()-p3.x())*(p2.x()-p4.x());
  double Num_y = (p1.y()-p3.y())*(p2.y()-p4.y());
  double Num_z = (p1.z()-p3.z())*(p2.z()-p4.z());
  double Den_x = (p1.x()-p4.x())*(p2.x()-p3.x());
  double Den_y = (p1.y()-p4.y())*(p2.y()-p3.y());
  double Den_z = (p1.z()-p4.z())*(p2.z()-p3.z());
  if (Den_x == Den_y && Den_y == Den_z) return (Num_x+Num_y+Num_z)/3/Den_x;
  else return (Den_x*Num_x+Den_y*Num_y+Den_z*Num_z)/(Den_x*Den_x+Den_y*Den_y+Den_z*Den_z);
}

//: Write "<vgl_point_3d x,y,z> " to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, vgl_point_3d<Type> const& p)
{
  return s << "<vgl_point_3d "<< p.x() << ',' << p.y() << ',' << p.z() << "> ";
}

//: Read from stream, possibly with formatting
//  Either just reads three blank-separated numbers,
//  or reads three comma-separated numbers,
//  or reads three numbers in parenthesized form "(123, 321, 567)"
// \relates vgl_point_3d
template <class Type>
vcl_istream& vgl_point_3d<Type>::read(vcl_istream& is)
{
  if (! is.good()) return is; // (TODO: should throw an exception)
  bool paren = false;
  Type x, y, z;
  is >> vcl_ws; // jump over any leading whitespace
  if (is.eof()) return is; // nothing to be set because of EOF (TODO: should throw an exception)
  if (is.peek() == '(') { is.ignore(); paren=true; }
  is >> vcl_ws >> x >> vcl_ws;
  if (is.eof()) return is;
  if (is.peek() == ',') is.ignore();
  is >> vcl_ws >> y >> vcl_ws;
  if (is.eof()) return is;
  if (is.peek() == ',') is.ignore();
  is >> vcl_ws >> z >> vcl_ws;
  if (paren) {
    if (is.eof()) return is;
    if (is.peek() == ')') is.ignore();
    else                  return is; // closing parenthesis is missing (TODO: throw an exception)
  }
  set(x,y,z);
  return is;
}

//: Read from stream, possibly with formatting
//  Either just reads three blank-separated numbers,
//  or reads three comma-separated numbers,
//  or reads three numbers in parenthesized form "(123, 321, 567)"
// \relates vgl_point_3d
template <class Type>
vcl_istream&  operator>>(vcl_istream& is, vgl_point_3d<Type>& p)
{
  return p.read(is);
}

#undef VGL_POINT_3D_INSTANTIATE
#define VGL_POINT_3D_INSTANTIATE(T) \
template class vgl_point_3d<T >; \
template double cross_ratio(vgl_point_3d<T >const&, vgl_point_3d<T >const&, \
                            vgl_point_3d<T >const&, vgl_point_3d<T >const&); \
template vcl_ostream& operator<<(vcl_ostream&, const vgl_point_3d<T >&); \
template vcl_istream& operator>>(vcl_istream&, vgl_point_3d<T >&)

#endif // vgl_point_3d_txx_
