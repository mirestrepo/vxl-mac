// This is vxl/vgl/vgl_box_2d.txx
#ifndef vgl_box_2d_txx_
#define vgl_box_2d_txx_

//:
// \file
// \brief Represents a cartesian 2D box.
// \author Don Hamilton, Peter Tu
// \date   Feb 15 2000

#include "vgl_box_2d.h"

#include <vcl_iostream.h>
#include <vcl_algorithm.h>
#include <vcl_cmath.h>
#include <vgl/vgl_point_2d.h>

// Constructors/Destructor---------------------------------------------------

template <class Type>
vgl_box_2d<Type>::vgl_box_2d(const Type min_position[2],
                             const Type max_position[2] )
{
  min_pos_[0]=min_position[0];
  min_pos_[1]=min_position[1];
  max_pos_[0]=max_position[0];
  max_pos_[1]=max_position[1];
}

template <class Type>
vgl_box_2d<Type>::vgl_box_2d(const vgl_point_2d<Type>& min_position,
                             const vgl_point_2d<Type>& max_position)
{
  min_pos_[0]=min_position.x();
  min_pos_[1]=min_position.y();
  max_pos_[0]=max_position.x();
  max_pos_[1]=max_position.y();
}

template <class Type>
vgl_box_2d<Type>::vgl_box_2d(Type xmin, Type xmax, Type ymin, Type ymax)
{
  min_pos_[0]=xmin;
  min_pos_[1]=ymin;
  max_pos_[0]=xmax;
  max_pos_[1]=ymax;
}

template <class Type>
vgl_box_2d<Type>::vgl_box_2d(const Type min_position[2],
                             Type width, Type height)
{
  min_pos_[0]=min_position[0];
  min_pos_[1]=min_position[1];
  max_pos_[0]=min_position[0]+width;
  max_pos_[1]=min_position[1]+height;
}

template <class Type>
vgl_box_2d<Type>::vgl_box_2d(const vgl_point_2d<Type>& min_position,
                             Type width, Type height)
{
  min_pos_[0]=min_position.x();
  min_pos_[1]=min_position.y();
  max_pos_[0]=min_position.x()+width;
  max_pos_[1]=min_position.y()+height;
}

template <class Type>
Type vgl_box_2d<Type>::centroid_x() const
{
  return (min_pos_[0] + max_pos_[0])/2;
}

template <class Type>
Type vgl_box_2d<Type>::centroid_y() const
{
  return (min_pos_[1] + max_pos_[1])/2;
}

template <class Type>
Type vgl_box_2d<Type>::width() const
{
  return (max_pos_[0] - min_pos_[0]);
}

template <class Type>
Type vgl_box_2d<Type>::height() const
{
  return (max_pos_[1] - min_pos_[1]);
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::min_point() const
{
  return vgl_point_2d<Type>(min_pos_[0],min_pos_[1]);
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::max_point() const
{
  return vgl_point_2d<Type>(max_pos_[0],max_pos_[1]);
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::centroid_point() const
{
  return vgl_point_2d<Type>(get_centroid_x(),get_centroid_y());
}

template <class Type>
void vgl_box_2d<Type>::set_centroid_x(Type centroid_x)
{
  Type delta = centroid_x - get_centroid_x();
  min_pos_[0]= min_pos_[0] + delta;
  max_pos_[0]= max_pos_[0] + delta;
}

template <class Type>
void vgl_box_2d<Type>::set_centroid_y(Type centroid_y)
{
  Type delta = centroid_y - get_centroid_y();
  min_pos_[1]= min_pos_[1] + delta;
  max_pos_[1]= max_pos_[1] + delta;
}

template <class Type>
void vgl_box_2d<Type>::set_width(Type width)
{
  Type x = get_centroid_x();
  min_pos_[0] = x-width/2;
  max_pos_[0] = x+width/2;
}

template <class Type>
void vgl_box_2d<Type>::set_height(Type height)
{
  Type y = get_centroid_y();
  min_pos_[1] = y-height/2;
  max_pos_[1] = y+height/2;
}

template <class Type>
void vgl_box_2d<Type>::setmin_position(Type min_position[2])
{
  min_pos_[0]=min_position[0];
  min_pos_[1]=min_position[1];
  if(max_pos_[0] < min_pos_[0]){
    max_pos_[0]=min_pos_[0];
  }
  if(max_pos_[1] < min_pos_[1]){
    max_pos_[1]=min_pos_[1];
  }
}

template <class Type>
void vgl_box_2d<Type>::setmax_position(Type max_position[2])
{
  max_pos_[0]=max_position[0];
  max_pos_[1]=max_position[1];
  if(max_pos_[0] < min_pos_[0]){
    min_pos_[0]=max_pos_[0];
  }
  if(max_pos_[1] < min_pos_[1]){
    min_pos_[1]=max_pos_[1];
  }
}

template <class Type>
void vgl_box_2d<Type>::set_min_point(vgl_point_2d<Type>& min_point)
{
  min_pos_[0]=min_point.x();
  min_pos_[1]=min_point.y();
  if(max_pos_[0] < min_pos_[0]){
    max_pos_[0]=min_pos_[0];
  }
  if(max_pos_[1] < min_pos_[1]){
    max_pos_[1]=min_pos_[1];
  }
}


template <class Type>
void vgl_box_2d<Type>::set_max_point(vgl_point_2d<Type>& max_point)
{
  max_pos_[0]=max_point.x();
  max_pos_[1]=max_point.y();
  if(max_pos_[0] < min_pos_[0]){
    min_pos_[0]=max_pos_[0];
  }
  if(max_pos_[1] < min_pos_[1]){
    min_pos_[1]=max_pos_[1];
  }
}

template <class Type>
void vgl_box_2d<Type>::set_centroid(Type centroid[2])
{
  set_centroid_x(centroid[0]);
  set_centroid_y(centroid[1]);
}

template <class Type>
void vgl_box_2d<Type>::set_centroid(vgl_point_2d<Type>& centroid)
{
  set_centroid_x(centroid.x());
  set_centroid_y(centroid.y());
}

template <class Type>
vcl_ostream& vgl_box_2d<Type>::print(vcl_ostream& s) const
{
  return s << "<vgl_box_2d "
       << min_pos_[0] << "," << min_pos_[1]
           << " to "
       << max_pos_[0] << "," << max_pos_[1]
       << ">";
}

template <class Type>
vcl_ostream& vgl_box_2d<Type>::write(vcl_ostream& s) const
{
  return s << min_pos_[0] << " " << min_pos_[1] << " "
       << max_pos_[0] << " " << max_pos_[1] << "\n";
}

template <class Type>
vcl_istream& vgl_box_2d<Type>::read(vcl_istream& s)
{
  return s >> min_pos_[0] >> min_pos_[1]
       >> max_pos_[0] >> max_pos_[1];
}

template <class Type>
vgl_box_2d<Type> intersect(vgl_box_2d<Type> const& a, vgl_box_2d<Type> const& b)
{
  Type x0 = vcl_max(a.get_min_x(), b.get_min_x());
  Type y0 = vcl_max(a.get_min_y(), b.get_min_y());
  Type x1 = vcl_min(a.get_max_x(), b.get_max_x());
  Type y1 = vcl_min(a.get_max_y(), b.get_max_y());

  if (x1 > x0 && y1 > y0)
    return vgl_box_2d<Type> (x0, x1, y0, y1);
  else
    return vgl_box_2d<Type> (0,0,0,0);

#if 0 // capes - replaced this wrong code
  return vgl_box_2d<Type>(vcl_max(a.get_min_x(), b.get_min_x()),
              vcl_min(a.get_max_x(), b.get_max_x()),
              vcl_max(a.get_min_y(), b.get_min_y()),
              vcl_min(a.get_max_y(), b.get_max_y())
              );
#endif
}

//: Print to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, const vgl_box_2d<Type>& p) {
  return p.print(s);
}

//: Read from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  vgl_box_2d<Type>& p) {
  return p.read(is);
}


// ---START DEPRECATED BLOCK---


template <class Type>
Type vgl_box_2d<Type>::get_centroid_x() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_centroid_x()");
  return centroid_x();
}

template <class Type>
Type vgl_box_2d<Type>::get_centroid_y() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_centroid_y()");
  return centroid_y();
}

template <class Type>
Type vgl_box_2d<Type>::get_width() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_width()");
  return width();
}

template <class Type>
Type vgl_box_2d<Type>::get_height() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_height()");
  return height();
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::get_min_point() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_min_point()");
  return min_point();
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::get_max_point() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_max_point()");
  return max_point();
}

template <class Type>
vgl_point_2d<Type> vgl_box_2d<Type>::get_centroid_point() const
{
  VXL_DEPRECATED("vgl_box_2d<T>::get_centroid_point()");
  return centroid_point();
}

// ---END DEPRECATED BLOCK---



#undef VGL_BOX_2D_INSTANTIATE
#define VGL_BOX_2D_INSTANTIATE(Type) \
template class vgl_box_2d<Type >;\
template vcl_istream& operator>>(vcl_istream&, vgl_box_2d<Type >&);\
template vcl_ostream& operator<<(vcl_ostream&, vgl_box_2d<Type > const&);\
template vgl_box_2d<Type > intersect(vgl_box_2d<Type > const&, vgl_box_2d<Type > const&)

#endif // vgl_box_2d_txx_
