// core/vil2/vil2_bicub_interp.txx
#ifndef vil2_bicub_interp_txx_
#define vil2_bicub_interp_txx_
//:
// \file
// \brief Bicubic interpolation functions for 2D images
//
// The vil2 bicub source files were derived from the corresponding
// vil2 bilin files, thus the vil2 bilin/bicub source files are very
// similar.  If you modify something in this file, there is a
// corresponding bilin file that would likely also benefit from
// the same change.
//
// In this particulat case, there is no corresponding
// vil2_bilin_interp.txx file, see vil2_bilin_interp.h instead.

#include "vil2_bicub_interp.h"

// vil2_bilin_interp.h defines only inline functions, but some of the
// corresponding vil2_bicub_interp functions are a little big to be
// inline.  Plus, on one platform, msvc 6.0 with /O2 optimization
// compiled without a peep but gave incorrect numerical results when
// these functions were inline and defined in vil2_bicub_interp.h.

template<class T>
double vil2_bicub_interp_unsafe(double x, double y, const T* data,
                                vcl_ptrdiff_t xstep, vcl_ptrdiff_t ystep)
{
    int p1x=int(x);
    double normx = x-p1x;
    int p1y=int(y);
    double normy = y-p1y;

    const T* pix1 = data + p1y*ystep + p1x*xstep;

    // like bilinear interpolation, use separability.
    // the s's are for the x-direction and the t's for the y-direction.
    double s0 = ((2-normx)*normx-1)*normx;    // -1
    double s1 = (3*normx-5)*normx*normx+2;    //  0
    double s2 = ((4-3*normx)*normx+1)*normx;  // +1
    double s3 = (normx-1)*normx*normx;        // +2

    double t0 = ((2-normy)*normy-1)*normy;
    double t1 = (3*normy-5)*normy*normy+2;
    double t2 = ((4-3*normy)*normy+1)*normy;
    double t3 = (normy-1)*normy*normy;

#define vil2_I(dx,dy) (pix1[(dx)*xstep+(dy)*ystep])

    double val = 0.25*
                 ( (s0*vil2_I(-1,-1) + s1*vil2_I(+0,-1) + s2*vil2_I(+1,-1) + s3*vil2_I(+2,-1))*t0 +
                   (s0*vil2_I(-1,+0) + s1*vil2_I(+0,+0) + s2*vil2_I(+1,+0) + s3*vil2_I(+2,+0))*t1 +
                   (s0*vil2_I(-1,+1) + s1*vil2_I(+0,+1) + s2*vil2_I(+1,+1) + s3*vil2_I(+2,+1))*t2 +
                   (s0*vil2_I(-1,+2) + s1*vil2_I(+0,+2) + s2*vil2_I(+1,+2) + s3*vil2_I(+2,+2))*t3 );

#undef vil2_I

    return val;
}

template<class T>
double vil2_bicub_interp_raw(double x, double y, const T* data,
                             vcl_ptrdiff_t xstep, vcl_ptrdiff_t ystep)
{
    int p1x=int(x);
    double normx = x-p1x;
    int p1y=int(y);
    double normy = y-p1y;

    const T* pix1 = data + p1y*ystep + p1x*xstep;

    // special boundary cases can be handled more quickly first; also
    // avoids accessing an invalid pix1[t] which is going to have
    // weight 0.

    if (normx == 0 && normy == 0) return pix1[0];

    // like bilinear interpolation, use separability.
    // the s's are for the x-direction and the t's for the y-direction.
    double s0 = ((2-normx)*normx-1)*normx;    // -1
    double s1 = (3*normx-5)*normx*normx+2;    //  0
    double s2 = ((4-3*normx)*normx+1)*normx;  // +1
    double s3 = (normx-1)*normx*normx;        // +2

#define vil2_I(dx,dy) (pix1[(dx)*xstep+(dy)*ystep])

    if (normy == 0)
        return 0.5 * (s0*vil2_I(-1,+0) + s1*vil2_I(+0,+0) + s2*vil2_I(+1,+0) + s3*vil2_I(+2,+0));

    double t0 = ((2-normy)*normy-1)*normy;
    double t1 = (3*normy-5)*normy*normy+2;
    double t2 = ((4-3*normy)*normy+1)*normy;
    double t3 = (normy-1)*normy*normy;

    // inefficiency: if normx is 0, then the s's were computed for nothing
    if (normx == 0)
        return 0.5 * (t0*vil2_I(+0,-1) + t1*vil2_I(+0,+0) + t2*vil2_I(+0,+1) + t3*vil2_I(+0,+2));

    double val = 0.25 *
                 ( (s0*vil2_I(-1,-1) + s1*vil2_I(+0,-1) + s2*vil2_I(+1,-1) + s3*vil2_I(+2,-1))*t0 +
                   (s0*vil2_I(-1,+0) + s1*vil2_I(+0,+0) + s2*vil2_I(+1,+0) + s3*vil2_I(+2,+0))*t1 +
                   (s0*vil2_I(-1,+1) + s1*vil2_I(+0,+1) + s2*vil2_I(+1,+1) + s3*vil2_I(+2,+1))*t2 +
                   (s0*vil2_I(-1,+2) + s1*vil2_I(+0,+2) + s2*vil2_I(+1,+2) + s3*vil2_I(+2,+2))*t3 );

#undef vil2_I

    return val;
}

#define VIL2_BICUB_INTERP_INSTANTIATE(T) \
template double \
vil2_bicub_interp_unsafe (double x, double y, const T* data, \
                          vcl_ptrdiff_t xstep, vcl_ptrdiff_t ystep); \
template double \
vil2_bicub_interp_raw (double x, double y, const T* data, \
                       vcl_ptrdiff_t xstep, vcl_ptrdiff_t ystep)

#endif // vil2_bicub_interp_txx_
