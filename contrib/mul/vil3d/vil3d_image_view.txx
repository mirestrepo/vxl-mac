// This is mul/vil3d/vil3d_image_view.txx
#ifndef vil3d_image_view_txx_
#define vil3d_image_view_txx_
//:
//  \file
//  \brief Represent images of one or more planes of Ts.
//  \author Tim Cootes, Ian Scott
//
// Note: To keep down size of vil3d_image_view
// Please think carefully before adding any new methods.
// In particular any methods that provide new views (e.g. vil3d_slice)
// will be more usefully provided as external functions. - IMS.
// In that case, use the "relates" keyword of Doxygen to link the documentation
// of that function to the vil3d_image_view class.

#include "vil3d_image_view.h"
#include <vcl_string.h>
#include <vcl_cassert.h>
#include <vcl_ostream.h>
#include <vil2/vil2_smart_ptr.h>
#include <vil2/vil2_pixel_format.h>

//=======================================================================

template<class T>
vil3d_image_view<T>::vil3d_image_view()
: top_left_(0),istep_(0),jstep_(0),kstep_(0),planestep_(0)
{}

template<class T>
vil3d_image_view<T>::vil3d_image_view(unsigned ni, unsigned nj,
                                      unsigned nk, unsigned n_planes)
: top_left_(0),istep_(1),jstep_(0),kstep_(0)
{
  set_size(ni,nj,nk,n_planes);
}

//: Set this view to look at someone else's memory data.
template<class T>
vil3d_image_view<T>::vil3d_image_view(const T* top_left,
                  unsigned n_i, unsigned n_j, unsigned n_k, unsigned n_planes,
                  int i_step, int j_step, int k_step, int plane_step)
{
  set_to_memory(top_left,n_i,n_j,n_k,n_planes,i_step,j_step,k_step,plane_step);
}

//: Set this view to look at another view's data
//  Need to pass the memory chunk to set up the internal smart ptr appropriately
template<class T>
vil3d_image_view<T>::vil3d_image_view(const vil2_memory_chunk_sptr& mem_chunk,
                    const T* top_left, unsigned n_i, unsigned n_j,
                    unsigned n_k, unsigned n_planes,
                    int i_step, int j_step, int k_step, int plane_step)
 : vil3d_image_view_base(n_i, n_j, n_k, n_planes)
 , top_left_(const_cast<T*>( top_left))
 , istep_(i_step), jstep_(j_step), kstep_(k_step)
 , planestep_(plane_step)
 , ptr_(mem_chunk)
{
  // check view and chunk are in rough agreement
  assert(mem_chunk->size() >= n_planes*n_i*n_j*n_k*sizeof(T));
  if (top_left < (const T*)mem_chunk->data() ||
      top_left >= (const T*)mem_chunk->data() + mem_chunk->size())
    vcl_cerr << "top_left at " << (const void*)top_left << ", memory_chunk at "
             << (const void*)mem_chunk->data() << ", size " << mem_chunk->size()
             << ", size of data type " << sizeof(T) << '\n';
  assert(top_left >= (const T*)mem_chunk->data() &&
         top_left  < (const T*)mem_chunk->data() + mem_chunk->size());
}

//: Copy constructor
// If this view cannot set itself to view the other data (e.g. because the
// types are incompatible) it will set itself to empty.
template<class T>
vil3d_image_view<T>::vil3d_image_view(const vil3d_image_view_base& that):
top_left_(0), istep_(0), jstep_(0), kstep_(0), planestep_(0), ptr_(0)
{
  operator=(that);
}

//: Sort of copy constructor
// If this view cannot set itself to view the other data (e.g. because the
// types are incompatible) it will set itself to empty.
template <class T>
vil3d_image_view<T>::vil3d_image_view(const vil3d_image_view_base_sptr& that):
top_left_(0), istep_(0), jstep_(0), kstep_(0), planestep_(0), ptr_(0)
{
  operator=(that);
}
//: Perform deep copy of the src image, placing in this image
template<class T>
void vil3d_image_view<T>::deep_copy(const vil3d_image_view<T>& src)
{
  set_size(src.ni(),src.nj(),src.nk(),src.nplanes());

  int s_planestep = src.planestep();
  int s_istep = src.istep();
  int s_jstep = src.jstep();
  int s_kstep = src.kstep();

  // Do a deep copy
  // This is potentially inefficient
  const T* src_data = src.origin_ptr();
  T* data = top_left_;
  for (unsigned int p=0;p<nplanes_;++p,src_data+=s_planestep,data+=planestep_)
  {
    T* slice = data;
    const T* src_slice = src_data;
    for (unsigned int k=0;k<nk_;++k,slice+=kstep_,src_slice+=s_kstep)
    {
      T* row = slice;
      const T* src_row = src_slice;
      for (unsigned int j=0;j<nj_;++j,row += jstep_,src_row += s_jstep)
      {
        T* p = row;
        const T* sp = src_row;
        for (unsigned int i=0;i<ni_;++i,p+=istep_,sp+=s_istep) *p = *sp;
      }
    }
  }
}

// Notes on convert_components_from_planes() and convert_planes_from_components()
// These are used by the operator= to provide the appropriate smart conversion
// behaviour for the various types.
// I don't think that C++ templates support full pattern matching,
// so we have to provide one template instantiation to cover the general
// compound pixel case (the range of which is possibly infinite)
// We then specialise for all the scalar pixel cases (there are only so
// many scalar types).
// I guess someone could merge all the scalar specialisations using
// macros and substantially reduce the length of this code.


//: Convert planes to components from planes, or do nothing if types are wrong.
template <class T>
inline bool convert_components_from_planes(vil3d_image_view<T> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  typedef typename T::value_type comp_type;

  const unsigned ncomp =
    vil2_pixel_format_num_components(vil2_pixel_format_of(T()));

  if (// both sides have equal component types and rhs has scalar pixels and
      rhs_base.pixel_format() == vil2_pixel_format_component_format(vil2_pixel_format_of(T()) ) &&
      // lhs has number of components equal to rhs's number of planes.
      ncomp == rhs_base.nplanes() )
  {
    const vil3d_image_view<comp_type> &rhs = static_cast<const vil3d_image_view<comp_type>&>(rhs_base);
    // Check that the steps are suitable for viewing as components
    if (rhs.planestep() != 1 || rhs.istep()%ncomp !=0 ||
        rhs.jstep()%ncomp !=0 || rhs.kstep()%ncomp !=0 ) return false;
    lhs = vil3d_image_view<T >(rhs.memory_chunk(),
                              (T const*) rhs.origin_ptr(),
                              rhs.ni(),rhs.nj(),rhs.nk(),1,
                              rhs.istep()/ncomp,rhs.jstep()/ncomp,rhs.kstep()/ncomp,1);
    return true;
  }
  else
    return false;
}


VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<float> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}  // when lhs has scalar pixels, don't attempt conversion

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<double> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<bool> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<vxl_byte> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<vxl_int_16> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<vxl_uint_16> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<vxl_int_32> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}

VCL_DEFINE_SPECIALIZATION
inline bool convert_components_from_planes(vil3d_image_view<vxl_uint_32> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{return false;}


//: Convert components to planes from planes, or do nothing if types are wrong.
template <class T>
inline bool convert_planes_from_components(vil3d_image_view<T> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{ return false;} // when lhs has non-scalar pixels, don't attempt conversion

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<vxl_byte> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_BYTE)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<vxl_byte> &rhs = static_cast<const vil3d_image_view<vxl_byte>&>(rhs_base);

    lhs = vil3d_image_view<vxl_byte>(rhs.memory_chunk(), rhs.origin_ptr(),
                                    rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                    rhs.istep()*ncomp,rhs.jstep()*ncomp,
                                    rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<vxl_uint_16> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_UINT_16)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<vxl_uint_16> &rhs = static_cast<const vil3d_image_view<vxl_uint_16>&>(rhs_base);

    lhs = vil3d_image_view<vxl_uint_16>(rhs.memory_chunk(), rhs.origin_ptr(),
                                       rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                       rhs.istep()*ncomp,rhs.jstep()*ncomp,rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<vxl_int_16> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_INT_16)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<vxl_int_16> &rhs = static_cast<const vil3d_image_view<vxl_int_16>&>(rhs_base);

    lhs = vil3d_image_view<vxl_int_16>(rhs.memory_chunk(), rhs.origin_ptr(),
                                      rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                      rhs.istep()*ncomp,rhs.jstep()*ncomp,rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<vxl_uint_32> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
       vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_UINT_32)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<vxl_uint_32> &rhs = static_cast<const vil3d_image_view<vxl_uint_32>&>(rhs_base);

    lhs = vil3d_image_view<vxl_uint_32>(rhs.memory_chunk(), rhs.origin_ptr(),
                                       rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                       rhs.istep()*ncomp,rhs.jstep()*ncomp,rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<vxl_int_32> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_INT_32)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<vxl_int_32> &rhs = static_cast<const vil3d_image_view<vxl_int_32>&>(rhs_base);

    lhs = vil3d_image_view<vxl_int_32>(rhs.memory_chunk(), rhs.origin_ptr(),
                                      rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                      rhs.istep()*ncomp,rhs.jstep()*ncomp,
                                      rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<float> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_FLOAT)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<float> &rhs = static_cast<const vil3d_image_view<float>&>(rhs_base);

    lhs = vil3d_image_view<float>(rhs.memory_chunk(), rhs.origin_ptr(),
                                 rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                 rhs.istep()*ncomp,rhs.jstep()*ncomp,rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}

VCL_DEFINE_SPECIALIZATION
inline bool convert_planes_from_components(vil3d_image_view<double> &lhs,
                                           const vil3d_image_view_base &rhs_base)
{
  const unsigned ncomp =
    vil2_pixel_format_num_components(rhs_base.pixel_format());

  if (// rhs has just 1 plane
      rhs_base.nplanes() == 1 &&
      // both sides have equal component types
      vil2_pixel_format_component_format(rhs_base.pixel_format()) == VIL2_PIXEL_FORMAT_DOUBLE)
  {
    // cheat by casting to component type, not pixel type (because we don't know full pixel type at compile time.)
    const vil3d_image_view<double> &rhs = static_cast<const vil3d_image_view<double>&>(rhs_base);

    lhs = vil3d_image_view<double>(rhs.memory_chunk(), rhs.origin_ptr(),
                                  rhs.ni(),rhs.nj(),rhs.nk(),ncomp,
                                  rhs.istep()*ncomp,rhs.jstep()*ncomp,rhs.kstep()*ncomp,1);
    return true;
  }
  else
    return false;
}


//: Create a copy of the data viewed by this, and return a view of copy.
// This function can be made a lot more powerful - to automatically convert between pixel types.
template<class T>
const vil3d_image_view<T> & vil3d_image_view<T>::operator= (const vil3d_image_view_base & rhs)
{
  if (static_cast<const vil3d_image_view_base*>(this) == &rhs)
    return *this;

  if (rhs.pixel_format() == pixel_format())
  {
    const vil3d_image_view<T> &that = static_cast<const vil3d_image_view<T>&>(rhs);
    ni_=that.ni_;
    nj_=that.nj_;
    nk_=that.nk_;
    nplanes_=that.nplanes_;
    istep_=that.istep_;
    jstep_=that.jstep_;
    kstep_=that.kstep_;
    planestep_=that.planestep_;
    top_left_=that.top_left_;
    ptr_=that.ptr_;
    return *this;
  }

  if (convert_components_from_planes(*this, rhs))
    return *this;

  if (convert_planes_from_components(*this, rhs))
    return *this;

  set_to_memory(0, 0, 0, 0, 0, 0, 0, 0,0);
  return *this;
}


//=======================================================================

template<class T> vil3d_image_view<T>::~vil3d_image_view()
{
  // release_data();
}

//=======================================================================


template<class T>
void vil3d_image_view<T>::set_size(unsigned n_i, unsigned n_j, unsigned n_k)
{
  set_size(n_i,n_j,n_k, nplanes_);
}

//: True if data all in one unbroken block and origin_ptr() is lowest data address
template<class T>
bool vil3d_image_view<T>::is_contiguous() const
{
  // RRR GGG BBB
  if (planestep_==int(ni_*nj_*nk_))
  {
    if (istep_==1 && jstep_==int(ni_) && kstep_==int(ni_*nj_) ) return true;
    if (istep_==1 && kstep_==int(ni_) && jstep_==int(ni_*nk_) ) return true;
    if (jstep_==1 && istep_==int(nj_) && kstep_==int(ni_*nj_) ) return true;
    if (jstep_==1 && kstep_==int(nj_) && istep_==int(nj_*nk_) ) return true;
    if (kstep_==1 && istep_==int(nk_) && jstep_==int(ni_*nk_) ) return true;
    if (kstep_==1 && jstep_==int(nk_) && istep_==int(nj_*nk_) ) return true;
  }

  int np = nplanes_;
  // RGBRGBRGB
  if (planestep_==1)
  {
    if (istep_==np && jstep_==int(ni_*np) && kstep_==int(ni_*nj_*np) ) return true;
    if (istep_==np && kstep_==int(ni_*np) && jstep_==int(ni_*nk_*np) ) return true;
    if (jstep_==np && istep_==int(nj_*np) && kstep_==int(ni_*nj_*np) ) return true;
    if (jstep_==np && kstep_==int(nj_*np) && istep_==int(nj_*nk_*np) ) return true;
    if (kstep_==np && istep_==int(nk_*np) && jstep_==int(ni_*nk_*np) ) return true;
    if (kstep_==np && jstep_==int(nk_*np) && istep_==int(nj_*nk_*np) ) return true;
  }

  // Note that there may be other weird combinations
  return false;
}

//=======================================================================

template<class T>
void vil3d_image_view<T>::set_size(unsigned n_i, unsigned n_j, unsigned n_k, unsigned n_planes)
{
  if (n_i==ni_ && n_j==nj_ && n_k==nk_ && n_planes==nplanes_) return;

  release_memory();

  ptr_ = new vil2_memory_chunk(sizeof(T)*n_planes*n_k*n_j*n_i,
    vil2_pixel_format_component_format(vil2_pixel_format_of(T())));

  ni_ = n_i;
  nj_ = n_j;
  nk_ = n_k;
  nplanes_ = n_planes;
  istep_ = 1;
  jstep_ = n_i;
  kstep_ = n_i*n_j;
  planestep_ = n_i*n_j*n_k;

  top_left_ = (T*) ptr_->data();
}


//: Set this view to look at someone else's memory.
template<class T>
void vil3d_image_view<T>::set_to_memory(const T* top_left,
                                       unsigned n_i, unsigned n_j,
                                       unsigned n_k, unsigned n_planes,
                                       int i_step, int j_step, int k_step, int plane_step)
{
  release_memory();
  top_left_ = const_cast<T*>(top_left);  // Remove const, as view may end up manipulating data

  ni_ = n_i;
  nj_ = n_j;
  nk_ = n_k;
  nplanes_ = n_planes;
  istep_ = i_step;
  jstep_ = j_step;
  kstep_ = k_step;
  planestep_ = plane_step;
}


//=======================================================================
//: Fill view with given value
template<class T>
void vil3d_image_view<T>::fill(T value)
{
  T* plane = top_left_;
  for (unsigned int p=0;p<nplanes_;++p,plane += planestep_)
  {
    T* slice = plane;
    for (unsigned int k=0;k<nk_;++k,slice += kstep_)
    {
      T* row = slice;
      for (unsigned int j=0;j<nj_;++j,row += jstep_)
      {
        T* p = row;
        for (unsigned int i=0;i<ni_;++i,p+=istep_) *p = value;
      }
    }
  }
}

//=======================================================================

template<class T>
bool vil3d_image_view<T>::is_class(vcl_string const& s) const
{
  return s==vil3d_image_view<T>::is_a() || vil3d_image_view_base::is_class(s);
}

//=======================================================================

template<class T>
void vil3d_image_view<T>::print(vcl_ostream& os) const
{
  os<<nplanes_<<" planes, each "<<ni_<<" x "<<nj_<<" x "<<nk_;
}

//=======================================================================
//: True if they share same view of same image data.
//  This does not do a deep equality on image data. If the images point
//  to different image data objects that contain identical images, then
//  the result will still be false.
template<class T>
bool vil3d_image_view<T>::operator==(const vil3d_image_view<T> &other) const
{
  if (!(bool) *this && !(bool)other) return true;
  return ptr_  == other.ptr_ &&
    top_left_  == other.top_left_ &&
    nplanes_   == other.nplanes_ &&
    ni_        == other.ni_ &&
    nj_        == other.nj_ &&
    nk_        == other.nk_ &&
    planestep_ == other.planestep_ &&
    istep_     == other.istep_ &&
    jstep_     == other.jstep_ &&
    kstep_     == other.kstep_;
}

//=======================================================================
//: Provides an ordering.
//  Useful for ordered containers.
//  There is no guaranteed meaning to the less than operator, except that
//  (a<b && b<a)  is false and  !(a<b) && !(b<a)  is equivalent to  a==b
template<class T>
bool vil3d_image_view<T>::operator<(const vil3d_image_view<T>& other) const
{
  if (ptr_ != other.ptr_) return ptr_<other.ptr_;
  if ((bool) *this && (bool)other) return false;
  if (nplanes_ != other.nplanes_) return nplanes_ < other.nplanes_;
  if (ni_ != other.ni_) return ni_ < other.ni_;
  if (nj_ != other.nj_) return nj_ < other.nj_;
  if (nk_ != other.nk_) return nk_ < other.nk_;
  if (planestep_ != other.planestep_) return planestep_ < other.planestep_;
  if (istep_ != other.istep_) return istep_ < other.istep_;
  if (jstep_ != other.jstep_) return jstep_ < other.jstep_;
  return kstep_ < other.kstep_;
}


//=======================================================================
//: True if the actual images are identical.
// $\bigwedge_{i,j,k,p} {\textstyle src}(i,j,k,p) == {\textstyle dest}(i,j,k,p)$
// The data may be formatted differently in each memory chunk.
//  O(size).
// \relates vil3d_image_view
template<class T>
bool vil3d_image_view_deep_equality(const vil3d_image_view<T> &lhs,
                                    const vil3d_image_view<T> &rhs)
{
  if (lhs.nplanes() != rhs.nplanes() ||
      lhs.nk() != rhs.nk() ||
      lhs.nj() != rhs.nj() ||
      lhs.ni() != rhs.ni())
    return false;

  for (unsigned p = 0; p < rhs.nplanes(); ++p)
   for (unsigned k = 0; k < rhs.nk(); ++k)
    for (unsigned j = 0; j < rhs.nj(); ++j)
      for (unsigned i = 0; i < rhs.ni(); ++i)
        if (!(rhs(i,j,k,p) == lhs(i,j,k,p)))
          return false;
  return true;
}

#define VIL3D_IMAGE_VIEW_INSTANTIATE(T) \
template class vil3d_image_view<T >; \
VCL_DEFINE_SPECIALIZATION vcl_string vil3d_image_view<T >::is_a() const \
{  return vcl_string("vil3d_image_view<" #T ">"); } \
template bool vil3d_image_view_deep_equality(const vil3d_image_view<T >&, \
                                            const vil3d_image_view<T >&);

#endif // vil3d_image_view_txx_
