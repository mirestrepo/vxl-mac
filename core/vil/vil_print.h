// This is core/vil/vil_print.h
#ifndef vil_print_h_
#define vil_print_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author Ian Scott, Tim Cootes.

#include <vil/vil_image_view.h>
#include <vcl_iostream.h>

//: How to print value in vil_print_all(image_view)
// \relates vil_image_view
template<class T>
void vil_print_value(vcl_ostream& s, const T& value);

//: Print all image data to os in a grid (rounds output to int)
// \relates vil_image_view
template<class T>
inline void vil_print_all(vcl_ostream& os,const vil_image_view<T>& view)
{
  os<<view.is_a()<<" "<<view.nplanes()<<" planes, each "<<view.ni()<<" x "<<view.nj()
    <<" istep: "<<(int)view.istep()<<' '
    <<" jstep: "<<(int)view.jstep()<<' '
    <<" planestep: "<<(int)view.planestep()<<'\n' << vcl_flush;
  for (unsigned int p=0;p<view.nplanes();++p)
  {
    if (view.nplanes()>1) os<<"Plane "<<p<<":\n" << vcl_flush;
    for (unsigned int j=0;j<view.nj();++j)
    {
      for (unsigned int i=0;i<view.ni();++i)
      {
        os<<' ';
        vil_print_value(os,view(i,j,p));
      }
      os<<'\n'<<vcl_flush;
    }
  }
}

//: Print all image data to os in a grid
// \relates vil_image_view
void vil_print_all(vcl_ostream& os, vil_image_view_base_sptr const& view);

#endif // vil_print_h_
