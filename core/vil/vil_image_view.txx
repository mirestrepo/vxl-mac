#ifndef vil2_image_view_txx_
#define vil2_image_view_txx_

//:
//  \file
//  \brief Represent images of one or more planes of Ts.
//  \author Ian Scott

#include "vil2_image_view.h"
#include <vcl_cstdlib.h>
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vil2/vil2_smart_ptr.h>
#include <vcl_ostream.h>

//=======================================================================

template<class T>
vil2_image_view<T>::vil2_image_view()
: top_left_(0),xstep_(0),ystep_(0),planestep_(0)
{}

template<class T>
vil2_image_view<T>::vil2_image_view(unsigned nx, unsigned ny, unsigned nplanes)
: top_left_(0),xstep_(1),ystep_(0)
{
  resize(nx,ny,nplanes);
}

//: Set this view to look at someone else's memory data.
template<class T>
vil2_image_view<T>::vil2_image_view(const T* top_left, unsigned nx, unsigned ny, unsigned nplanes,
                  unsigned xstep, unsigned ystep, unsigned planestep)
{
  set_to_memory(top_left,nx,ny,nplanes,xstep,ystep,planestep);
}

//: Set this view to look at another view's data
//  Need to pass the memory chunk to set up the internal smart ptr appropriately
template<class T>
vil2_image_view<T>::vil2_image_view(const vil2_smart_ptr<vil2_memory_chunk>& mem_chunk,
                  const T* top_left, unsigned nx, unsigned ny, unsigned nplanes,
                  unsigned xstep, unsigned ystep, unsigned planestep)
{
  ptr_ = mem_chunk;
  set_to_memory(top_left,nx,ny,nplanes,xstep,ystep,planestep);
}


//: Perform deep copy of the src image, placing in this image
template<class T>
void vil2_image_view<T>::deep_copy(const vil2_image_view<T>& src)
{
  resize(src.nx(),src.ny(),src.nplanes());

  int s_planestep = src.planestep();
  int s_xstep = src.xstep();
  int s_ystep = src.ystep();

  // Do a deep copy
  // This is potentially inefficient
  const T* src_data = src.top_left_ptr();
  T* data = top_left_;
  for (unsigned int i=0;i<nplanes_;++i)
  {
    T* row = data;
    const T* src_row = src_data;
    for (int y=0;y<ny_;++y)
    {
      T* p = row;
      const T* sp = src_row;
      for (int x=0;x<nx_;++x)
      {
        *p = *sp;
		p+=xstep_;
        sp+=s_xstep;
      }
      row += ystep_;
      src_row += s_ystep;
    }
	src_data += s_planestep;
	data += planestep_;
  }
}

//: Create a copy of the data viewed by this, and return a view of copy.
template<class T>
vil2_image_view<T> vil2_image_view<T>::deep_copy() const
{
 vil2_image_view<T> cpy;
 cpy.deep_copy(*this);
 return cpy;
}

//=======================================================================

template<class T>
void vil2_image_view<T>::release_data()
{
  ptr_=0;
}

template<class T> vil2_image_view<T>::~vil2_image_view()
{
  // release_data();
}

//=======================================================================


template<class T>
void vil2_image_view<T>::resize(unsigned nx, unsigned ny)
{
  resize(nx,ny, nplanes_);
}

//: True if data all in one unbroken block
template<class T>
bool vil2_image_view<T>::is_contiguous() const
{
  // RRR GGG BBB
  if (planestep_==nx_*ny_)
  {
    if (xstep_==1 && ystep_==nx_)  return true;
    if (ystep_==1 && xstep_==ny_) return true;
  }

  // RGBRGBRGB
  if (planestep_==1)
  {
    if (xstep_==nplanes_ && ystep_==nx_*nplanes_)  return true;
    if (ystep_==nplanes_ && xstep_==ny_*nplanes_)  return true;
  }

  // Note that there may be other weird combinations
  return false;
}

//=======================================================================

template<class T>
void vil2_image_view<T>::resize(unsigned nx, unsigned ny, unsigned nplanes)
{
  if (nx==nx_ && ny==ny_ && nplanes==nplanes_) return;

  release_data();

  ptr_ = new vil2_memory_chunk(sizeof(T)*nplanes*ny*nx);

  nx_ = nx;
  ny_ = ny;
  nplanes_ = nplanes;
  xstep_ = 1;
  ystep_ = nx;
  planestep_ = nx*ny;

  top_left_ = (T*) ptr_->data();
}


//: Set this view to look at someone else's memory.
template<class T>
void vil2_image_view<T>::set_to_memory(const T* top_left,
                             unsigned nx, unsigned ny, unsigned nplanes,
                             unsigned xstep, unsigned ystep, unsigned planestep)
{
  release_data();
  top_left_ = (T*) top_left;  // Remove const, as view may end up manipulating data

  nx_ = nx;
  ny_ = ny;
  nplanes_ = nplanes;
  xstep_ = xstep;
  ystep_ = ystep;
  planestep_ = planestep;
}



//=======================================================================
//: Arrange that this is window on given image.
//  I.e. plane(i) points to im.plane(i+p0) + offset
template<class T>
void vil2_image_view<T>::set_to_window(const vil2_image_view& im,
                     unsigned x0, unsigned nx, unsigned y0,
                     unsigned ny, unsigned p0, unsigned np)
{
  assert(this!=&im);

  assert(x0<im.nx()); assert(x0+nx<=im.nx());
  assert(y0<im.ny()); assert(y0+ny<=im.ny());
  assert(p0<im.nplanes()); assert(p0+np<=im.nplanes());

  release_data();

  // Take smart pointer to im's data to keep it in scope
  ptr_ = im.ptr_;

  nx_ = nx;
  ny_ = ny;
  nplanes_ = np;
  xstep_ = im.xstep();
  ystep_ = im.ystep();
  planestep_ = im.planestep();

    // Have to force the cast to avoid compiler warnings about const
  top_left_ = (T*) im.top_left_ptr() + x0*xstep_ + y0*ystep_ + p0*planestep_;
}

//: Arrange that this is window on all planes of given image.
template<class T>
void vil2_image_view<T>::set_to_window(const vil2_image_view& im,
                     unsigned x0, unsigned nx, unsigned y0, unsigned ny)
{
  set_to_window(im,x0,ny,y0,ny,0,im.nplanes());
}

//: Return an nx x ny window of this data with offset (x0,y0)
template<class T>
vil2_image_view<T> vil2_image_view<T>::window(unsigned x0, unsigned nx, unsigned y0, unsigned ny) const
{
  vil2_image_view<T> win;
  win.set_to_window(*this,x0,nx,y0,ny);
  return win;
}

//: Return a view of plane p
template<class T>
vil2_image_view<T> vil2_image_view<T>::plane(int p) const
{
  vil2_image_view<T> p_view;
  p_view.set_to_window(*this,0,nx_,0,ny_,p,1);
  return p_view;
}

//: Create a view which appears as the transpose of this view
//  ie transpose()(x,y,p) = this(y,x,p)
template<class T>
vil2_image_view<T> vil2_image_view<T>::transpose() const
{
  vil2_image_view<T> t = *this;
  // Swap x and y
  t.nx_    = ny_;
  t.xstep_ = ystep_;
  t.ny_    = nx_;
  t.ystep_ = xstep_;
  return t;
}

//: Fill view with given value
template<class T>
void vil2_image_view<T>::fill(T value)
{
  T* plane = top_left_;
  for (unsigned int i=0;i<nplanes_;++i,plane += planestep_)
  {
    T* row = plane;
    for (int y=0;y<ny_;++y,row += ystep_)
    {
      T* p = row;
      for (int x=0;x<nx_;++x,p+=xstep_) *p = value;
    }
  }
}

//=======================================================================

template<class T>
bool vil2_image_view<T>::is_class(vcl_string const& s) const
{
  return s==vil2_image_view<T>::is_a() || vil2_image_view_base::is_class(s);
}

//=======================================================================

template<class T>
void vil2_image_view<T>::print(vcl_ostream& os) const
{
  os<<nplanes_<<" planes, each "<<nx_<<" x "<<ny_;
}

//: print all data to os
template<class T>
void vil2_image_view<T>::print_all(vcl_ostream& os) const
{
  print(os);
  os<<"  xstep: "<<xstep_<<" ystep: "<<ystep_<<" planestep: "<<planestep_<<vcl_endl;

  for (int i=0;i<nplanes();++i)
  {
    if (nplanes()>1) os<<"Plane "<<i<<":"<<vcl_endl;
    const T* im_data = top_left_ + i*planestep_;
    for (int y=0;y<ny_;++y)
    {
      for (int x=0;x<nx_;++x)
      {
        int v = int(im_data[ystep_*y+x*xstep_]);
        if (v<10)  os<<" ";
        if (v<100) os<<" ";
        os<<v<<" ";
      }
      os<<vcl_endl;
    }
  }
}


//=======================================================================
//: True if they share same view of same image data.
//  This does not do a deep equality on image data. If the images point
//  to different image data objects that contain identical images, then
//  the result will still be false.
template<class T>
bool vil2_image_view<T>::operator==(const vil2_image_view<T> &other) const
{
  return ptr_  == other.ptr_ &&
    top_left_  == other.top_left_ &&
    nplanes_   == other.nplanes_ &&
    nx_        == other.nx_ &&
    ny_        == other.ny_ &&
    planestep_ == other.planestep_ &&
    xstep_     == other.xstep_ &&
    ystep_     == other.ystep_;
}


#define VIL2_IMAGE_VIEW_INSTANTIATE(T) \
template class vil2_image_view<T >

#endif // vil2_image_view_txx_
