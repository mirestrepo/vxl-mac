#ifndef mil3d_gaussian_pyramid_builder_3d_txx_
#define mil3d_gaussian_pyramid_builder_3d_txx_

//: \file
//  \brief Class to build gaussian pyramids of mil3d_image_3d_of<T>
//  \author Tim Cootes

#include "mil3d_gaussian_pyramid_builder_3d.h"

#include <vcl_cstdlib.h>
#include <vcl_string.h>

#include <mil/mil_image_pyramid.h>
#include <mil3d/mil3d_gauss_reduce_3d.h>
#include <vcl_cassert.h>
#include <vcl_cmath.h>

//=======================================================================

template<class T>
mil3d_gaussian_pyramid_builder_3d<T>::mil3d_gaussian_pyramid_builder_3d()
: max_levels_(99),filter_width_(5)
{
  set_min_size(5, 5, 5);
}

//=======================================================================

template<class T>
mil3d_gaussian_pyramid_builder_3d<T>::~mil3d_gaussian_pyramid_builder_3d()
{
}

//=======================================================================
//: Define maximum number of levels to build
//  Limits levels built in subsequent calls to build()
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::setMaxLevels(int max_l)
{
  if (max_l<1)
  {
    vcl_cerr<<"mil3d_gaussian_pyramid_builder_3d<T>::setMaxLevels() ";
    vcl_cerr<<"Must be >=1\n";
    vcl_abort();
  }
  max_levels_ = max_l;
}

//: Get the current maximum number levels allowed
template<class T>
int mil3d_gaussian_pyramid_builder_3d<T>::maxLevels() const
{
  return max_levels_;
}

//=======================================================================
//: Create new (empty) pyramid on heap
//  Caller responsible for its deletion
template<class T>
mil_image_pyramid* mil3d_gaussian_pyramid_builder_3d<T>::newImagePyramid() const
{
  return new mil_image_pyramid;
}


//=======================================================================
//: Scale step between levels
template<class T>
double mil3d_gaussian_pyramid_builder_3d<T>::scale_step() const
{
  return 2.0;
}

//: Set current filter width (must be 3 or 5 at present)
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::set_filter_width(unsigned w)
{
  assert(w==5);
  filter_width_ = w;
}


//=======================================================================
//: Smooth and subsample src_im to produce dest_im
//  Applies filter in x and y, then samples every other pixel.
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::gauss_reduce_15851(mil3d_image_3d_of<T>& dest_im,
                                                            const mil3d_image_3d_of<T>& src_im) const
{
  int nx = src_im.nx();
  int ny = src_im.ny();
  int nz = src_im.nz();
  int n_planes = src_im.n_planes();

  // Output image size
  int nx2 = (nx+1)/2;
  int ny2 = (ny+1)/2;
  int nz2 = (nz+1)/2;

  if (dest_im.n_planes()!=n_planes)
    dest_im.set_n_planes(n_planes);
  dest_im.resize(nx2,ny2,nz2);

vcl_cout<<"Generating new image of size "<<nx2<<" x "<<ny2<<" x "<<nz2<<vcl_endl;
vcl_cout<<"Dest im zstep = "<<dest_im.zstep()<<vcl_endl;
vcl_cout<<dest_im<<vcl_endl;

  if (work_im1_.nx()<nx2 || work_im1_.ny()<ny || work_im1_.nz()<nz)
    work_im1_.resize(nx2,ny,nz);

  if (work_im2_.nx()<nx2 || work_im2_.ny()<ny2 || work_im2_.nz()<nz)
    work_im2_.resize(nx2,ny2,nz);

  // Reduce plane-by-plane
  for (int i=0;i<n_planes;++i)
  {
    // Smooth and subsample in x, result in work_im1_
    mil3d_gauss_reduce_3d(work_im1_.plane(0),work_im1_.xstep(),work_im1_.ystep(),work_im1_.zstep(),
      src_im.plane(i),nx,ny,nz,
      src_im.xstep(),src_im.ystep(),src_im.zstep());

    // Smooth and subsample in y (by implicitly transposing), result in work_im2_
    mil3d_gauss_reduce_3d(work_im2_.plane(0),work_im2_.ystep(),work_im2_.xstep(),work_im2_.zstep(),
      work_im1_.plane(0),ny,nx2,nz,
      work_im1_.ystep(),work_im1_.xstep(),work_im1_.zstep());

    // Smooth and subsample in z (by implicitly transposing)
    mil3d_gauss_reduce_3d(dest_im.plane(i),dest_im.zstep(),dest_im.xstep(),dest_im.ystep(),
      work_im2_.plane(0),nz,nx2,ny2,
      work_im2_.zstep(),work_im2_.xstep(),work_im2_.ystep());
  }

  // Sort out world to image transformation for destination image
  mil3d_transform_3d scaling;
  scaling.set_zoom_only(0.5,0,0,0);
  dest_im.setWorld2im(scaling * src_im.world2im());
}

//=======================================================================
//: Smooth and subsample src_im to produce dest_im
//  Applies 1-5-8-5-1 filter in x and y, then samples
//  every other pixel.
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::gauss_reduce(mil3d_image_3d_of<T>& dest_im,
                                                      const mil3d_image_3d_of<T>& src_im) const
{
  switch (filter_width_)
  {
    case (5):
      gauss_reduce_15851(dest_im,src_im);
      break;
    default:
      vcl_cerr<<"mil3d_gaussian_pyramid_builder_3d<T>::gauss_reduce() ";
      vcl_cerr<<"Cannot cope with filter width of "<<filter_width_<<vcl_endl;
      vcl_abort();
  }
}

//=======================================================================
//: Deletes all data in im_pyr
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::emptyPyr(mil_image_pyramid& im_pyr) const
{
  for (int i=0; i<im_pyr.n_levels();++i)
    delete im_pyr.data()[i];
}

//=======================================================================
//: Checks pyramid has at least n levels
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::checkPyr(mil_image_pyramid& im_pyr,  int n_levels) const
{
  const int got_levels = im_pyr.n_levels();
  if (got_levels >= n_levels && im_pyr(0).is_a()==work_im1_.is_a())
  {
    if (im_pyr.n_levels()==n_levels) return;
    else
    {
      for (int i=n_levels;i<got_levels;++i)
        delete im_pyr.data()[i];
    }
    im_pyr.data().resize(n_levels);
    return;
  }

  im_pyr.data().resize(n_levels);
  emptyPyr(im_pyr);

  for (int i=0;i<n_levels;++i)
    im_pyr.data()[i] = new mil3d_image_3d_of<T>;
}

//: Set the minimum size of the top layer of the pyramid
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::set_min_size(unsigned X, unsigned Y, unsigned Z)
{ min_x_size_ = X; min_y_size_ = Y; min_z_size_ = Z;}

//=======================================================================
//: Build pyramid
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::build(mil_image_pyramid& image_pyr,
                                               const mil_image& im) const
{
  //  Require image mil3d_image_3d_of<T>
  assert(im.is_a()==work_im1_.is_a());

  const mil3d_image_3d_of<T>& base_image = (const mil3d_image_3d_of<T>&) im;

  int nx = base_image.nx();
  int ny = base_image.ny();
  int nz = base_image.nz();

  // Compute number of levels to pyramid so that top is no less
  // than min_x_size_ x min_y_size_ x minZsize_
  int s = 1;
  int max_levels = 1;
  while ((nx/(2*s)>=int(min_x_size_)) && (ny/(2*s)>=int(min_y_size_)) && (nz/(2*s)>=int(min_z_size_)))
  {
    max_levels++;
    s*=2;
  }

  if (max_levels>max_levels_)
    max_levels=max_levels_;

  work_im1_.resize(nx,ny,nz);
  work_im2_.resize(nx,ny,nz);

  // Set up image pyramid
  checkPyr(image_pyr,max_levels);

  mil3d_image_3d_of<T>& im0 = (mil3d_image_3d_of<T>&) image_pyr(0);

  // Shallow copy of part of base_image
  im0.setToWindow(base_image,0,nx-1,0,ny-1,0,nz-1);

  int i;
  for (i=1;i<max_levels;i++)
  {
    mil3d_image_3d_of<T>& im_i0 = (mil3d_image_3d_of<T>&) image_pyr(i);
    mil3d_image_3d_of<T>& im_i1 = (mil3d_image_3d_of<T>&) image_pyr(i-1);

    gauss_reduce(im_i0,im_i1);
  }

  // Estimate width of pixels in base image
  vgl_point_3d<double>  c0(0.5*(nx-1),0.5*(ny-1),0.5*(nz-1));
  vgl_point_3d<double>  c1 = c0 + vgl_vector_3d<double> (1,1,1);
  mil3d_transform_3d im2world = base_image.world2im().inverse();
  vgl_vector_3d<double>  dw = im2world(c1) - im2world(c0);

  double base_pixel_width = dw.length()/vcl_sqrt(3.0);
  double scale_step = 2.0;

  image_pyr.setWidths(base_pixel_width,scale_step);
}

//=======================================================================
//: Extend pyramid
template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::extend(mil_image_pyramid& image_pyr) const
{
  //  Require image mil3d_image_3d_of<T>
  assert(image_pyr(0).is_a() == work_im1_.is_a());

  assert(image_pyr.scale_step() == scale_step());

  int nx = image_pyr(0).nx();
  int ny = image_pyr(0).ny();
  int nz = image_pyr(0).nz();

  // Compute number of levels to pyramid so that top is no less
  // than 5 x 5
  double s = 1;
  int max_levels = 1;
  while ((nx/(scale_step()*s)>=min_x_size_) && (ny/(scale_step()*s)>=min_x_size_) && (nz/(2*s)>=int(min_z_size_)))
  {
    max_levels++;
    s*=scale_step();
  }

  if (max_levels>max_levels_)
    max_levels=max_levels_;

  work_im1_.resize(nx,ny,nz);
  work_im2_.resize(nx,ny,nz);

  // Set up image pyramid
  int oldsize = image_pyr.n_levels();
  if (oldsize<max_levels) // only extend, if it isn't already tall enough
  {
    image_pyr.data().resize(max_levels);

    int i;
    for (i=oldsize;i<max_levels;++i)
      image_pyr.data()[i] = new mil3d_image_3d_of<T>;

    for (i=oldsize;i<max_levels;i++)
    {
      mil3d_image_3d_of<T>& im_i0 = (mil3d_image_3d_of<T>&) image_pyr(i);
      mil3d_image_3d_of<T>& im_i1 = (mil3d_image_3d_of<T>&) image_pyr(i-1);

      gauss_reduce(im_i0,im_i1);
    }
  }
}
//=======================================================================

#if 0
template<class T>
vcl_string mil3d_gaussian_pyramid_builder_3d<T>::is_a() const
{
  return vcl_string("mil3d_gaussian_pyramid_builder_3d<T>");
}
#endif

//=======================================================================

template<class T>
bool mil3d_gaussian_pyramid_builder_3d<T>::is_class(vcl_string const& s) const
{
  return s==mil3d_gaussian_pyramid_builder_3d<T>::is_a() || mil_image_pyramid_builder::is_class(s);
}

//=======================================================================

template<class T>
short mil3d_gaussian_pyramid_builder_3d<T>::version_no() const
{
  return 2;
}

//=======================================================================

template<class T>
mil_image_pyramid_builder* mil3d_gaussian_pyramid_builder_3d<T>::clone() const
{
  return new mil3d_gaussian_pyramid_builder_3d<T>(*this);
}

//=======================================================================

template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::print_summary(vcl_ostream&) const
{
}

//=======================================================================

template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no());
  vsl_b_write(bfs,max_levels_);
  vsl_b_write(bfs,filter_width_);
}

//=======================================================================

template<class T>
void mil3d_gaussian_pyramid_builder_3d<T>::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;

  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
  case (1):
    vsl_b_read(bfs,max_levels_);
    filter_width_=5;
    break;
  case (2):
    vsl_b_read(bfs,max_levels_);
    vsl_b_read(bfs,filter_width_);
    break;
  default:
    vcl_cerr << "I/O ERROR: mil3d_gaussian_pyramid_builder_3d<T>::b_read(vsl_b_istream&)\n";
    vcl_cerr << "           Unknown version number "<< version << "\n";
    bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    return;
  }
}

#endif // mil3d_gaussian_pyramid_builder_3d_txx_
