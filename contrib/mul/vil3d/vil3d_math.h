// This is mul/vil3d/vil3d_math.h
#ifndef vil3d_math_h_
#define vil3d_math_h_
//:
// \file
// \brief Various mathematical manipulations of 3D images
// \author Tim Cootes

#include <vcl_cassert.h>
#include <vil3d/vil3d_image_view.h>
#include <vil3d/vil3d_plane.h>

//: Compute minimum and maximum values over im
// \relates vil3d_image_view
template <class T>
inline void vil3d_math_value_range(const vil3d_image_view<T>& im,
                                   T& min_value, T& max_value)
{
  if (im.size()==0)
  {
    min_value = 0;
    max_value = 0;
    return;
  }

  const T* plane = im.origin_ptr();
  min_value = *plane;
  max_value = min_value;

  unsigned ni = im.ni();
  unsigned nj = im.nj();
  unsigned nk = im.nk();
  unsigned np = im.nplanes();
  vcl_ptrdiff_t istep = im.istep(), jstep=im.jstep(), kstep=im.kstep();

  for (unsigned int p=0;p<np;++p, plane += im.planestep())
  {
    const T* slice = plane;
    for (unsigned int k=0;k<nk;++k, slice += kstep)
    {
      const T* row = slice;
      for (unsigned int j=0;j<nj;++j, row += jstep)
      {
        const T* pixel = row;
        for (unsigned int i=0;i<ni;++i, pixel+=istep)
        {
          if (*pixel<min_value)      min_value = *pixel;
          else if (*pixel>max_value) max_value = *pixel;
        }
      }
    }
  }
}

//: Compute sum of values in plane p
// \relates vil3d_image_view
template <class imT, class sumT>
inline void vil3d_math_sum(sumT& sum, const vil3d_image_view<imT>& im,
                           unsigned p)
{
  assert(p<im.nplanes());
  sum=0;
  if (im.size()==0)
  {
    return;
  }

  const imT* plane = im.origin_ptr();
  unsigned ni = im.ni();
  unsigned nj = im.nj();
  unsigned nk = im.nk();
  vcl_ptrdiff_t istep = im.istep(), jstep=im.jstep(), kstep=im.kstep();

  const imT* slice = plane + p*im.planestep();
  for (unsigned int k=0;k<nk;++k, slice += kstep)
  {
    const imT* row = slice;
    for (unsigned int j=0;j<nj;++j, row += jstep)
    {
      const imT* pixel = row;
      for (unsigned int i=0;i<ni;++i, pixel+=istep) sum += sumT(*pixel);
    }
  }
}

//: Mean of elements in plane p of image
// \relates vil3d_image_view
template <class imT, class sumT>
inline void vil3d_math_mean(sumT& mean, const vil3d_image_view<imT>& im,
                            unsigned p)
{
  if (im.size()==0) { mean=0; return; }
  vil3d_math_sum(mean,im,p);
  mean/=(im.ni()*im.nj()*im.nk());
}


//: Sum of squares of elements in plane p of image
// \relates vil3d_image_view
template <class imT, class sumT>
inline void vil3d_math_sum_squares(sumT& sum, sumT& sum_sq,
                              const vil3d_image_view<imT>& im, unsigned p)
{
  assert(p<im.nplanes());
  sum = 0; sum_sq=0;
  if (im.size()==0)
  {
    return;
  }

  const imT* plane = im.origin_ptr();
  unsigned ni = im.ni();
  unsigned nj = im.nj();
  unsigned nk = im.nk();
  vcl_ptrdiff_t istep = im.istep(), jstep=im.jstep(), kstep=im.kstep();

  const imT* slice = plane + p*im.planestep();
  for (unsigned int k=0;k<nk;++k, slice += kstep)
  {
    const imT* row = slice;
    for (unsigned int j=0;j<nj;++j, row += jstep)
    {
      const imT* p = row;
      for (unsigned int i=0;i<ni;++i, p+=istep)
      { sum += *p; sum_sq+=sumT(*p)*sumT(*p); }
    }
  }
}

//: Mean and variance of elements in plane p of image
// \relates vil3d_image_view
template <class imT, class sumT>
inline void vil3d_math_mean_and_variance(sumT& mean, sumT& var,
                                         const vil3d_image_view<imT>& im,
                                         unsigned p)
{
  if (im.size()==0) { mean=0; var=0; return; }
  sumT sum, sum_sq;
  vil3d_math_sum_squares(sum,sum_sq,im,p);
  mean = sum/(im.ni()*im.nj()*im.nk());
  var = sum_sq/(im.ni()*im.nj()*im.nk()) - mean*mean;
}

#endif
