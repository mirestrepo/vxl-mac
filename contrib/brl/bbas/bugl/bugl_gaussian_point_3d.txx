#ifndef bugl_gaussian_point_3d_txx_
#define bugl_gaussian_point_3d_txx_

#include "bugl_gaussian_point_3d.h"
#include <vnl/vnl_inverse.h>
#include <vnl/vnl_det.h>
#include <vnl/vnl_math.h>

template<class T>
bugl_gaussian_point_3d<T>::bugl_gaussian_point_3d(T x, T y, T z, vnl_matrix_fixed<T, 3, 3> & s)
: bugl_uncertainty_point_3d<T>(x, y, z)
{
  set_covariant_matrix(s);
}

template<class T>
bugl_gaussian_point_3d<T>::bugl_gaussian_point_3d(vgl_point_3d<T> &p, vnl_matrix_fixed<T, 3, 3> &s)
: bugl_uncertainty_point_3d<T>(p)
{
  set_covariant_matrix(s);
}

template<class T>
void bugl_gaussian_point_3d<T>::set_covariant_matrix(vnl_matrix_fixed<T, 3, 3> &s)
{
  sigma_ = s;
  sigma_inv_ = vnl_inverse(s);
  det_ = vnl_det(s);
}

template<class T>
bugl_gaussian_point_3d<T>::bugl_gaussian_point_3d() :
bugl_uncertainty_point_3d<T>()
{
}

template<class T>
T bugl_gaussian_point_3d<T>::prob_at(vgl_point_3d<T> &p)
{
  vnl_vector_fixed<T, 3> d(p.x() - this->x(), p.y() - y(), p.z() - z());

  const double pi = vnl_math::pi;
  return vcl_exp(-0.5*(dot_product(d, sigma_inv_*d)))/vcl_sqrt(8*pi)/pi/det_;
}

#if 0
template<class T>
bugl_gaussian_point_3d<T>& bugl_gaussian_point_3d<T>::operator=(bugl_gaussian_point_3d<T> &p)
{
  vnl_matrix_fixed<T, 3, 3> t = p.get_covariant_matrix();
  set_covariant_matrix(t);
  set_point(p);
  return *this;
}
#endif

//----------------------------------------------------------------------------
#undef BUGL_GAUSSIAN_POINT_3D_INSTANTIATE
#define BUGL_GAUSSIAN_POINT_3D_INSTANTIATE(T) \
template class bugl_gaussian_point_3d<T >

#endif // bugl_gaussian_point_3d_txx_
