// This is vxl/vgl/vgl_homg_line_2d.txx
#ifndef vgl_homg_line_2d_txx_
#define vgl_homg_line_2d_txx_

#include <vcl_iostream.h>
#include "vgl_homg_line_2d.h"
#include <vgl/vgl_homg_point_2d.h>
#include <vgl/vgl_line_2d.h>

template <class Type>
vgl_homg_line_2d<Type>::vgl_homg_line_2d (vgl_line_2d<Type> const& l)
 : a_(l.a()) , b_(l.b()) , c_(l.c())
{
}

//: get two points on the line.  These two points are normally the intersections
// with the Y axis and X axis, respectively.  When the line is parallel to one
// of these, the point with y=1 or x=1, resp. are taken.  When the line goes
// through the origin, the second point is (b, -a, 1).  Finally, when the line
// is the line at infinity, the returned points are (1,0,0) and (0,1,0).
// Thus, whenever possible, the returned points are not at infinity.
template <class Type>
void vgl_homg_line_2d<Type>::get_two_points(vgl_homg_point_2d<Type> &p1, vgl_homg_point_2d<Type> &p2)
{
  if (this->b() == 0) p1.set(-this->c(), this->a(), this->a());
  else                p1.set(0, -this->c(), this->b());
  if (this->a() == 0) p2.set(this->b(), -this->c(), this->b());
  else if ( c() == 0) p2.set(this->b(), -this->a(), 1);
  else                p2.set(-this->c(), 0, this->a());
}

template <class Type>
vgl_homg_line_2d<Type>::vgl_homg_line_2d (vgl_homg_point_2d<Type> const& p1,
                                          vgl_homg_point_2d<Type> const& p2)
{
  set(p1.y()*p2.w()-p1.w()*p2.y(),
      p1.w()*p2.x()-p1.x()*p2.w(),
      p1.x()*p2.y()-p1.y()*p2.x());
  assert(a_||b_||c_); // given points should be different
}

template <class Type>
bool vgl_homg_line_2d<Type>::operator==(vgl_homg_line_2d<Type> const& other) const
{
  return (this==&other) ||
         (   this->a()*other.c() == this->c()*other.a()
          && this->b()*other.c() == this->c()*other.b());
}

//: Print line equation to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, vgl_homg_line_2d<Type>const& p) {
  return s << " <vgl_homg_line_2d "
           << p.a() << " x + " << p.b() << " y + "
           << p.c() << " w = 0>";
}

//: Load in line parameters from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& is, vgl_homg_line_2d<Type>& p) {
  Type a,b,c;
  is >> a >> b >> c;
  p.set(a,b,c);
  return is;
}

#undef VGL_HOMG_LINE_2D_INSTANTIATE
#define VGL_HOMG_LINE_2D_INSTANTIATE(T) \
template class vgl_homg_line_2d<T >; \
template vcl_ostream& operator<<(vcl_ostream&, vgl_homg_line_2d<T >const&); \
template vcl_istream& operator>>(vcl_istream&, vgl_homg_line_2d<T >&)

#endif // vgl_homg_line_2d_txx_
