#ifndef vil_cartesian_differential_invariants_txx_
#define vil_cartesian_differential_invariants_txx_

//:
// \file
// \brief Find Cartesian differential Invariants
// \author Ian Scott
// Based on Some Matlab code by K. Walker 1999.

#include <vil/algo/vil_cartesian_differential_invariants.h>
#include <vil/vil_image_view.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_convolve_1d.h>
#include <vil/vil_math.h>
#include <vil/vil_transpose.h>


template <class T> T sqr(T) { return v*v; }

//: Compute 1st, 2nd, and 3rd order C.d.i.s of an image.
// The input must be 1 plane, the output will be 8 planes.
void vil_cartesian_differential_invariants_3(
  const vil_image_view<float>& src, vil_image_view<float>& dest, double scale)
{
  assert(src.nplanes()==1);
  const unsigned filt_n=7;
  const unsigned filt_c=filt_n/2;

  vcl_vector<double> filt_0(filt_n), filt_2(filt_n), filt_1(filt_n), filt_3(filt_n);
  vil_gauss_filter_gen_ntap(scale, 0, filt_0);
  vil_gauss_filter_gen_ntap(scale, 1, filt_1);
  vil_gauss_filter_gen_ntap(scale, 2, filt_2);
  vil_gauss_filter_gen_ntap(scale, 3, filt_3);
  
  const vil_convolve_boundary_option bo = vil_convolve_constant_extend;

  // Filter all the x directions
  vil_image_view<float> LX, LXx, LXxx, LXxxx;
  vil_convolve_1d(src, LX, &filt_0[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(src, LXx, &filt_1[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(src, LXxx, &filt_2[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(src, LXxxx, &filt_3[filt_c], -filt_c, filt_c, double(), bo, bo);
  
  // Now calculate the full values.
  vil_image_view<float> Lx, Ly, Lxx, Lxy, Lyy, Lxxx, Lxxy, Lxyy, Lyyy;

  // construct first order partial derivatives
  vil_convolve_1d(vil_transpose(LXx), Lx,
    &filt_0[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LX), Ly,
    &filt_1[filt_c], -filt_c, filt_c, double(), bo, bo);

  // construct second order partial derivatives
  vil_convolve_1d(vil_transpose(LXxx), Lxx,
    &filt_0[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LXx), Lxy,
    &filt_1[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LX), Lyy,
    &filt_2[filt_c], -filt_c, filt_c, double(), bo, bo);

  // construct third order partial derivatives
  vil_convolve_1d(vil_transpose(LXxxx), Lxxx,
    &filt_0[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LXxx), Lxxy,
    &filt_1[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LXx), Lxyy,
    &filt_2[filt_c], -filt_c, filt_c, double(), bo, bo);
  vil_convolve_1d(vil_transpose(LX), Lyyy,
    &filt_3[filt_c], -filt_c, filt_c, double(), bo, bo);

  dest.set_size(src.ni(), src.nj(), 8);
  for (unsigned j=0;j<dest.ni();++j)
  {
    for (unsigned i=0;i<dest.nj();++i)
    {
      // I1 = LiLi=LxLx+LyLy
      // cdi(1,:,:)=(Lx.*Lx)+(Ly.*Ly);
      dest(j,i,0)= (Lx(i,j)*Lx(i,j))
                  +(Ly(i,j)*Ly(i,j));

      // LiLijLj=LxxLxLx+2LxLxyLy+LyyLyLy
      // cdi(2,:,:)=(Lxx.*Lx.*Lx)+(2.*Lx.*Lxy.*Ly)+(Lyy.*Ly.*Ly);
      dest(j,i,1)= (Lxx(i,j)*Lx(i,j)*Lx(i,j))
                  +(2 *Lx(i,j)*Lxy(i,j)*Ly(i,j))
                  +(Lyy(i,j)*Ly(i,j)*Ly(i,j));

      // %LiiLjLj-LijLiLj=LxxLyLy-2LxyLxLy+LyyLxLx
      // cdi(3,:,:)=(Lxx.*Ly.*Ly)-(2.*Lxy.*Lx.*Ly)+(Lyy.*Lx.*Lx);
      dest(j,i,2)= (Lxx(i,j)*Ly(i,j)*Ly(i,j))
                  -(2.0*Lxy(i,j)*Lx(i,j)*Ly(i,j))
                  +(Lyy(i,j)*Lx(i,j)*Lx(i,j));


      // %-eijLjkLiLk=LxxLxLy+LxyLyLy-LyyLxLy-LxyLxLx
      // cdi(4,:,:)=(Lxx.*Lx.*Ly)+(Lxy.*Ly.*Ly)-(Lyy.*Lx.*Ly)-(Lxy.*Lx.*Lx);
      dest(j,i,3)= (Lxx(i,j)*Lx(i,j)*Ly(i,j))
                  +(Lxy(i,j)*Ly(i,j)*Ly(i,j))
                  -(Lyy(i,j)*Lx(i,j)*Ly(i,j))
                  -(Lxy(i,j)*Lx(i,j)*Lx(i,j));

      // %eij(LjklLiLkLl-LjkkLiLlLl)=(2LxyyLxLxLy)-(2LxxyLxLyLy)-(LxxyLxLyLy)-(LyyyLxLxLx)+(LxyyLyLxLx)+(LxxxLyLyLy)
      // cdi(5,:,:)=(2.*Lxyy.*Lx.*Lx.*Ly)-(2.*Lxxy.*Lx.*Ly.*Ly)-(Lxxy.*Lx.*Ly.*Ly)-(Lyyy.*Lx.*Lx.*Lx)+(Lxyy.*Ly.*Lx.*Lx)+(Lxxx.*Ly.*Ly.*Ly);
      dest(j,i,4)= (2.0*Lxyy(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j))
                  -(2.0*Lxxy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  -(Lxxy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  -(Lyyy(i,j)*Lx(i,j)*Lx(i,j)*Lx(i,j))
                  +(Lxyy(i,j)*Ly(i,j)*Lx(i,j)*Lx(i,j))
                  +(Lxxx(i,j)*Ly(i,j)*Ly(i,j)*Ly(i,j));


      // %LiijLjLkLk-LijkLiLjLk=(LxxxLxLyLy)-(2LxxyLxLxLy)+(LxxyLyLyLy)+(LxyyLxLxLx)-(2LxyyLxLyLy)+(LyyylxLxLy)
      // cdi(6,:,:)=(Lxxx.*Lx.*Ly.*Ly)-(2.*Lxxy.*Lx.*Lx.*Ly)+(Lxxy.*Ly.*Ly.*Ly)+(Lxyy.*Lx.*Lx.*Lx)-(2.*Lxyy.*Lx.*Ly.*Ly)+(Lyyy.*Lx.*Lx.*Ly);
      dest(j,i,5)= (Lxxx(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  -(2.0*Lxxy(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j))
                  +(Lxxy(i,j)*Ly(i,j)*Ly(i,j)*Ly(i,j))
                  +(Lxyy(i,j)*Lx(i,j)*Lx(i,j)*Lx(i,j))
                  -(2.0*Lxyy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  +(Lyyy(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j));


      // %-eijLjklLiLkLl=(LxxxLxLxLy)+(2LxxyLxLyLy)+(LxyyLyLyLy)-(LyyyLxLyLy)-(2LxyyLxLxLy)-(LxxyLxLxLx)
      // cdi(7,:,:)=(Lxxx.*Lx.*Lx.*Ly)+(2.*Lxxy.*Lx.*Ly.*Ly)+(Lxyy.*Ly.*Ly.*Ly)-(Lyyy.*Lx.*Ly.*Ly)-(2.*Lxyy.*Lx.*Lx.*Ly)-(Lxxy.*Lx.*Lx.*Lx);
      dest(j,i,6)= (Lxxx(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j))
                  +(2.0*Lxxy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  +(Lxyy(i,j)*Ly(i,j)*Ly(i,j)*Ly(i,j))
                  -(Lyyy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j))
                  -(2.0*Lxyy(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j))
                  -(Lxxy(i,j)*Lx(i,j)*Lx(i,j)*Lx(i,j));


      // %LijkLiLjLk=(LxxxLxLxLx)+(LyyyLyLyLy)+(3LxxyLxLxLy)+(3LxyyLxLyLy)
      // cdi(8,:,:)=(Lxxx.*Lx.*Lx.*Lx)+(Lyyy.*Ly.*Ly.*Ly)+(3.*Lxxy.*Lx.*Lx.*Ly)+(3.*Lxyy.*Lx.*Ly.*Ly);
      dest(j,i,7)= (Lxxx(i,j)*Lx(i,j)*Lx(i,j)*Lx(i,j))
                  +(Lyyy(i,j)*Ly(i,j)*Ly(i,j)*Ly(i,j))
                  +(3.0*(i,j)*Lxxy(i,j)*Lx(i,j)*Lx(i,j)*Ly(i,j))
                  +(3.0*(i,j)*Lxyy(i,j)*Lx(i,j)*Ly(i,j)*Ly(i,j));
    }
  }
}


#endif // vil_cartesian_differential_invariants_h_
