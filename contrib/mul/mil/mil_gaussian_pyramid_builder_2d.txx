#ifndef mil_gaussian_pyramid_builder_2d_txx_
#define mil_gaussian_pyramid_builder_2d_txx_

//: \file
//  \brief Class to build gaussian pyramids of mil_image_2d_of<T>
//  \author Tim Cootes

#include <vcl_cstdlib.h>
#include <vcl_string.h>

#include <vsl/vsl_indent.h>
#include <mil/mil_gaussian_pyramid_builder_2d.h>
#include <mil/mil_image_pyramid.h>
#include <mil/mil_gauss_reduce_2d.h>
#include <vcl_cassert.h>
#include <vcl_cmath.h>
#include <vnl/vnl_math.h> // for sqrt2

//=======================================================================

template<class T>
mil_gaussian_pyramid_builder_2d<T>::mil_gaussian_pyramid_builder_2d()
    : max_levels_(99)
{
	set_min_size(5, 5);
}

//=======================================================================

template<class T>
mil_gaussian_pyramid_builder_2d<T>::~mil_gaussian_pyramid_builder_2d()
{
}

//=======================================================================
//: Define maximum number of levels to build
//  Limits levels built in subsequent calls to build()
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::setMaxLevels(int max_l)
{
    if (max_l<1)
    {
        vcl_cerr<<"mil_gaussian_pyramid_builder_2d<T>::setMaxLevels() ";
        vcl_cerr<<"Must be >=1"<<vcl_endl;
        vcl_abort();
    }
    max_levels_ = max_l;
}

//: Get the current maximum number levels allowed
template<class T>
int mil_gaussian_pyramid_builder_2d<T>::maxLevels()
{
  return max_levels_;
}

//=======================================================================
//: Create new (empty) pyramid on heap
//  Caller responsible for its deletion
template<class T>
mil_image_pyramid* mil_gaussian_pyramid_builder_2d<T>::newImagePyramid() const
{
    return new mil_image_pyramid;
}


//=======================================================================
//: Scale step between levels
template<class T>
double mil_gaussian_pyramid_builder_2d<T>::scale_step() const
{
    return 2.0;
}

//=======================================================================
//: Smooth and subsample src_im to produce dest_im
//  Applies 1-5-8-5-1 filter in x and y, then samples
//  every other pixel.
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::gauss_reduce(mil_image_2d_of<T>& dest_im,
                     const mil_image_2d_of<T>& src_im)
{
    int nx = src_im.nx();
    int ny = src_im.ny();
    int n_planes = src_im.n_planes();

    // Output image size
    int nx2 = (nx+1)/2;
    int ny2 = (ny+1)/2;

    if (dest_im.n_planes()!=n_planes)
        dest_im.set_n_planes(n_planes);
    dest_im.resize(nx2,ny2);

    if (work_im_.nx()<nx2 || work_im_.ny()<ny)
         work_im_.resize(nx2,ny);

        // Reduce plane-by-plane
    for (int i=0;i<n_planes;++i)
    {
      // Smooth and subsample in x, result in work_im
      mil_gauss_reduce_2d(work_im_.plane(0),work_im_.xstep(),work_im_.ystep(),
                      src_im.plane(i),nx,ny,
                      src_im.xstep(),src_im.ystep());

      // Smooth and subsample in y (by implicitly transposing)
      mil_gauss_reduce_2d(dest_im.plane(i),dest_im.ystep(),dest_im.xstep(),
                      work_im_.plane(0),ny,nx2,
                      work_im_.ystep(),work_im_.xstep());
    }

        // Sort out world to image transformation for destination image
    mil_transform_2d scaling;
    scaling.set_zoom_only(0.5,0,0);
    dest_im.setWorld2im(scaling * src_im.world2im());
}

//=======================================================================
//: Deletes all data in im_pyr
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::emptyPyr(mil_image_pyramid& im_pyr)
{
    for (int i=0; i<im_pyr.nLevels();++i)
        delete im_pyr.data()[i];
}

//=======================================================================
//: Checks pyramid has at least n levels
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::checkPyr(mil_image_pyramid& im_pyr,  int n_levels)
{
    if (im_pyr.nLevels()>=n_levels)
    {
        if (im_pyr(0).is_a()==work_im_.is_a()) return;
    }

    emptyPyr(im_pyr);
    im_pyr.data().resize(n_levels);

    for (int i=0;i<n_levels;++i)
        im_pyr.data()[i] = new mil_image_2d_of<T>;
}

//=======================================================================
//: Build pyramid
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::build(mil_image_pyramid& image_pyr,
                                    const mil_image& im)
{
    //  Require image mil_image_2d_of<T>
    assert(im.is_a()==work_im_.is_a());

    const mil_image_2d_of<T>& base_image = (const mil_image_2d_of<T>&) im;

    int nx = base_image.nx();
    int ny = base_image.ny();

    // Compute number of levels to pyramid so that top is no less
    // than minXSize_ x minYSize_
    int s = 1;
    int max_levels = 1;
    while ((nx/(2*s)>=minXSize_) && (ny/(2*s)>=minYSize_))
    {
        max_levels++;
        s*=2;
    }

    if (max_levels>max_levels_)
        max_levels=max_levels_;

    work_im_.resize(nx,ny);

    // Set up image pyramid
    checkPyr(image_pyr,max_levels);

    mil_image_2d_of<T>& im0 = (mil_image_2d_of<T>&) image_pyr(0);

    // Shallow copy of part of base_image
    im0.setToWindow(base_image,0,nx-1,0,ny-1);

    int i;
    for (i=1;i<max_levels;i++)
    {
        mil_image_2d_of<T>& im_i0 = (mil_image_2d_of<T>&) image_pyr(i);
        mil_image_2d_of<T>& im_i1 = (mil_image_2d_of<T>&) image_pyr(i-1);

        gauss_reduce(im_i0,im_i1);
    }

    // Estimate width of pixels in base image
    vgl_point_2d<double>  c0(0.5*(nx-1),0.5*(ny-1));
    vgl_point_2d<double>  c1 = c0 + vgl_vector_2d<double> (1,1);
    mil_transform_2d im2world = base_image.world2im().inverse();
    vgl_vector_2d<double>  dw = im2world(c1) - im2world(c0);

    double base_pixel_width = dw.length()/vnl_math::sqrt2;
    double scale_step = 2.0;

    image_pyr.setWidths(base_pixel_width,scale_step);
}

//=======================================================================
//: Extend pyramid
// The first layer of the pyramid must already be set.
template<class T>
void mil_gaussian_pyramid_builder_2d<T>::extend(mil_image_pyramid& image_pyr)
{
    
    //  Require image mil_image_2d_of<T>
    assert(image_pyr(0).is_a() == work_im_.is_a());

    assert(image_pyr.scale_step() == scale_step());

    int nx = image_pyr(0).nx();
    int ny = image_pyr(0).ny();

    // Compute number of levels to pyramid so that top is no less
    // than 5 x 5
    int s = 1;
    int max_levels = 1;
    while ((nx/(scale_step()*s)>=minXSize_) && (ny/(scale_step()*s)>=minXSize_))
    {
        max_levels++;
        s*=scale_step();
    }

    if (max_levels>max_levels_)
        max_levels=max_levels_;

    work_im_.resize(nx,ny);

    // Set up image pyramid
    int oldsize = image_pyr.nLevels();
    if (oldsize<max_levels) // only extend, if it isn't already tall enough
    {
      image_pyr.data().resize(max_levels);


      int i;
      for (i=oldsize;i<max_levels;++i)
          image_pyr.data()[i] = new mil_image_2d_of<T>;

      for (i=oldsize;i<max_levels;i++)
      {
          mil_image_2d_of<T>& im_i0 = (mil_image_2d_of<T>&) image_pyr(i);
          mil_image_2d_of<T>& im_i1 = (mil_image_2d_of<T>&) image_pyr(i-1);

          gauss_reduce(im_i0,im_i1);
      }
    }
}


//=======================================================================

template<class T>
vcl_string mil_gaussian_pyramid_builder_2d<T>::is_a() const
{
  return vcl_string("mil_gaussian_pyramid_builder_2d<T>");
}

//=======================================================================

template<class T>
bool mil_gaussian_pyramid_builder_2d<T>::is_class(vcl_string const& s) const
{
  static const vcl_string s_ = "mil_gaussian_pyramid_builder_2d<T>";
  return s==s_ || mil_image_pyramid_builder::is_class(s);
}

//=======================================================================

template<class T>
short mil_gaussian_pyramid_builder_2d<T>::version_no() const
{
    return 1;
}

//=======================================================================

template<class T>
mil_image_pyramid_builder* mil_gaussian_pyramid_builder_2d<T>::clone() const
{
    return new mil_gaussian_pyramid_builder_2d<T>(*this);
}

//=======================================================================

template<class T>
void mil_gaussian_pyramid_builder_2d<T>::print_summary(vcl_ostream&) const
{
}

//=======================================================================

template<class T>
void mil_gaussian_pyramid_builder_2d<T>::b_write(vsl_b_ostream& bfs) const
{
    vsl_b_write(bfs,version_no());
    vsl_b_write(bfs,max_levels_);
}

//=======================================================================

template<class T>
void mil_gaussian_pyramid_builder_2d<T>::b_read(vsl_b_istream& bfs)
{
    short version;
    vsl_b_read(bfs,version);
    switch (version)
    {
        case (1):
            vsl_b_read(bfs,max_levels_);
            break;
        default:
            vcl_cerr << "mil_gaussian_pyramid_builder_2d<T>::b_read() ";
            vcl_cerr << "Unexpected version number " << version << vcl_endl;
            vcl_abort();
    }
}

#endif // mil_gaussian_pyramid_builder_2d_txx_
