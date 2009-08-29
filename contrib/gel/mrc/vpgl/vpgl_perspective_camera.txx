// This is gel/mrc/vpgl/vpgl_perspective_camera.txx
#ifndef vpgl_perspective_camera_txx_
#define vpgl_perspective_camera_txx_
//:
// \file

#include "vpgl_perspective_camera.h"
#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_homg_plane_3d.h>
#include <vgl/vgl_line_3d_2_points.h>
#include <vgl/algo/vgl_h_matrix_3d.h>
#include <vnl/vnl_det.h>
#include <vnl/algo/vnl_qr.h>
#include <vnl/vnl_matrix_fixed.h>

#include <vgl/io/vgl_io_point_3d.h>
#include <vnl/io/vnl_io_matrix_fixed.h>
#include <vnl/io/vnl_io_vector_fixed.h>

//-------------------------------------------
template <class T>
vpgl_perspective_camera<T>::vpgl_perspective_camera()
{
  R_.set_identity();
  camera_center_.set( (T)0, (T)0, (T)0 );
  recompute_matrix();
}


//-------------------------------------------
template <class T>
vpgl_perspective_camera<T>::vpgl_perspective_camera(
  const vpgl_calibration_matrix<T>& K,
  const vgl_point_3d<T>& camera_center,
  const vgl_rotation_3d<T>& R ) :
  K_( K ), camera_center_( camera_center ), R_( R )
{
  recompute_matrix();
}

//-------------------------------------------
template <class T>
vpgl_perspective_camera<T>::vpgl_perspective_camera(
  const vpgl_calibration_matrix<T>& K,
  const vgl_rotation_3d<T>& R,
  const vgl_vector_3d<T>& t) :
  K_( K ),  R_( R )
{
  this->set_translation(t);
  recompute_matrix();
}

//-------------------------------------------
template <class T>
vpgl_perspective_camera<T>::vpgl_perspective_camera( const vpgl_perspective_camera& that)
  : vpgl_proj_camera<T>(that),
  K_(that.K_),
  camera_center_(that.camera_center_),
  R_(that.R_)
{
}

//-------------------------------------------
template <class T>
vpgl_proj_camera<T>* vpgl_perspective_camera<T>::clone(void) const
{
  return new vpgl_perspective_camera<T>(*this);
}


//------------------------------------
template <class T>
vgl_line_3d_2_points<T> vpgl_perspective_camera<T>::backproject(
  const vgl_point_2d<T>& image_point ) const
{
  // First find a point in front of the camera that projects to "image_point".
  vnl_vector_fixed<T,4> vnl_wp = this->svd()->solve(
    vnl_vector_fixed<T,3>( image_point.x(), image_point.y(), 1.0 ) );
  vgl_homg_point_3d<T> wp_homg( vnl_wp[0], vnl_wp[1], vnl_wp[2], vnl_wp[3] );
  vgl_point_3d<T> wp;
  if ( !wp_homg.ideal() )
    wp.set( wp_homg.x()/wp_homg.w(), wp_homg.y()/wp_homg.w(), wp_homg.z()/wp_homg.w() );
  else
    wp.set( camera_center_.x()+wp_homg.x(),
            camera_center_.y()+wp_homg.y(),
            camera_center_.z()+wp_homg.z() );
  if ( is_behind_camera( vgl_homg_point_3d<T>( wp ) ) )
    wp = camera_center_ + ( camera_center_-wp );

  // The ray is then defined by that point and the camera center.
  return vgl_line_3d_2_points<T>( camera_center_, wp );
}


//-------------------------------------------
template <class T>
vgl_vector_3d<T> vpgl_perspective_camera<T>::principal_axis() const
{
  // See H&Z pg 147
  // P = [M|p4] : We do not need to compute det(M) because we enforce
  // det(K)>0 and det(R)=1 in the construction of P. Thus det(M)>0;
  const vnl_matrix_fixed<T,3,4>& P = this->get_matrix();
  return normalized(vgl_vector_3d<T>(P(2,0),P(2,1),P(2,2)));
}


//------------------------------------
template <class T>
bool vpgl_perspective_camera<T>::is_behind_camera(
  const vgl_homg_point_3d<T>& world_point ) const
{
  vgl_homg_plane_3d<T> l = this->principal_plane();
  T dot = world_point.x()*l.a() + world_point.y()*l.b() +
               world_point.z()*l.c() + world_point.w()*l.d();
  if (world_point.w() < (T)0) dot = ((T)(-1))*dot;
  return dot < 0;
}


//-------------------------------------------
template <class T>
void vpgl_perspective_camera<T>::set_calibration( const vpgl_calibration_matrix<T>& K)
{
  K_ = K;
  recompute_matrix();
}


//-------------------------------------------
template <class T>
void vpgl_perspective_camera<T>::set_camera_center(
  const vgl_point_3d<T>& camera_center )
{
  camera_center_ = camera_center;
  recompute_matrix();
}

//-------------------------------------------
template <class T>
void vpgl_perspective_camera<T>::set_translation(const vgl_vector_3d<T>& t)
{
  vgl_rotation_3d<T> Rt = R_.transpose();
  vgl_vector_3d<T> cv = -(Rt * t);
  camera_center_.set(cv.x(), cv.y(), cv.z());
  recompute_matrix();
}

//-------------------------------------------
template <class T>
void vpgl_perspective_camera<T>::set_rotation( const vgl_rotation_3d<T>& R )
{
  R_ = R;
  recompute_matrix();
}

//-------------------------------------------
template <class T>
vgl_vector_3d<T> vpgl_perspective_camera<T>::get_translation() const
{
  vgl_vector_3d<T> c(camera_center_.x(), camera_center_.y(),camera_center_.z());
  vgl_vector_3d<T> temp = R_*c;
  return -temp;
}

//: Rotate the camera about its center such that it looks at the given point
//  The camera should also be rotated about its principal axis such that
//  the vertical image direction is closest to \p up in the world
template <class T>
void vpgl_perspective_camera<T>::look_at(const vgl_homg_point_3d<T>& point,
                                         const vgl_vector_3d<T>& up )
{
  vgl_vector_3d<T> u = normalized(up);
  vgl_vector_3d<T> look = point - camera_center();
  normalize(look);
  T dp = dot_product(look, up);
  bool singularity = vcl_fabs(vcl_fabs(static_cast<double>(dp))-1.0)<1e-08;
  assert(!singularity);
  vgl_vector_3d<T> z = look;
  vgl_vector_3d<T> x = cross_product(-u,z);
  vgl_vector_3d<T> y = cross_product(z,x);
  normalize(x);
  normalize(y);
  normalize(z);

  T r[] = { x.x(), x.y(), x.z(),
            y.x(), y.y(), y.z(),
            z.x(), z.y(), z.z() };

  vnl_matrix_fixed<T,3,3> R(r);
  set_rotation(vgl_rotation_3d<T>(R));
}


//-------------------------------------------
template <class T>
void vpgl_perspective_camera<T>::recompute_matrix()
{
    vnl_matrix_fixed<T,3,4> Pnew( (T)0 );

   // Set new projection matrix to [ I | -C ].
   for ( int i = 0; i < 3; i++ )
     Pnew(i,i) = (T)1;
   Pnew(0,3) = -camera_center_.x();
   Pnew(1,3) = -camera_center_.y();
   Pnew(2,3) = -camera_center_.z();

   // Then multiply on left to get KR[ I | -C ].
   this->set_matrix(K_.get_matrix()*R_.as_matrix()*Pnew);
}


//-------------------------------------------
template <class T>
bool vpgl_perspective_decomposition( const vnl_matrix_fixed<T,3,4>& camera_matrix,
                                     vpgl_perspective_camera<T>& p_camera )
{
  // Extract the left sub matrix H from [ H t ] and check that it has rank 3.
  vnl_matrix_fixed<T,3,3> H = camera_matrix.extract(3,3);
  vnl_vector_fixed<T,3> t = camera_matrix.get_column(3);

  T det = vnl_det(H);
  if ( det == (T)0 ) return false;
  // To insure a true rotation (determinant = 1) we must start with a positive
  // determinant H.  This is decomposed into K and R, each with positive determinant.
  if ( det < (T)0 ){
    H *= (T)-1;
    t *= (T)-1;
  }

  // Now find the RQ decomposition of the sub matrix and use these to find the params.
  // This will involve some trickery as VXL has no RQ decomposition, but does have QR.
  // Define a matrix "flipping" operation by f(A)ij = An-j,n-i i.e. f flips the matrix
  // about its LL-UR diagonal.  One can prove that if f(A) = B*C then A = f(A)*f(B). So
  // we get the RQ decomposition of A by flipping A, then taking the QR decomposition,
  // then flipping both back, noting that the flipped Q and R will remain orthogonal and
  // upper right triangular respectively.
  vnl_matrix_fixed<T,3,3> Hf;
  for ( int i = 0; i < 3; i++ )
    for ( int j = 0; j < 3; j++ )
      Hf(i,j) = H(2-j,2-i);
  vnl_qr<T> QR( Hf );
  vnl_matrix_fixed<T,3,3> q,r,Qf,Rf;
  q = QR.Q();
  r = QR.R();
  for ( int i = 0; i < 3; i++ ){
    for ( int j = 0; j < 3; j++ ){
      Qf(i,j) = q(2-j,2-i);
      Rf(i,j) = r(2-j,2-i);
    }
  }

  // We almost have the K and R parameter blocks, but we must be sure that the diagonal
  // entries of K are positive.
  int r0pos = Rf(0,0) > 0 ? 1 : -1;
  int r1pos = Rf(1,1) > 0 ? 1 : -1;
  int r2pos = Rf(2,2) > 0 ? 1 : -1;
  int diag[3] = { r0pos, r1pos, r2pos };
  vnl_matrix_fixed<T,3,3> K1,R1;
  for ( int i = 0; i < 3; i++ ){
    for ( int j = 0; j < 3; j++ ){
      K1(i,j) = diag[j]*Rf(i,j);
      R1(i,j) = diag[i]*Qf(i,j);
    }
  }
  K1 = K1/K1(2,2);

  // Now we extract the parameters from the matrices we've computed;
  vpgl_calibration_matrix<T> new_K( K1 );
  p_camera.set_calibration( new_K );

  vnl_qr<T> QRofH(H);
  vnl_vector<T> c1 = -QRofH.solve(t);
  vgl_point_3d<T> new_c( c1(0), c1(1), c1(2) );
  p_camera.set_camera_center( new_c );

  p_camera.set_rotation( vgl_rotation_3d<T>(R1) );

  return true;
}


//-------------------------------------------
template <class T>
vpgl_perspective_camera<T> vpgl_align_down(
  const vpgl_perspective_camera<T>& p0,
  const vpgl_perspective_camera<T>& p1 )
{
  vpgl_perspective_camera<T> new_camera;
  new_camera.set_calibration( p0.get_calibration() );
  new_camera.set_rotation( p1.get_rotation()*p0.get_rotation().inverse() );
  vgl_point_3d<T> a0 = p0.get_rotation()*p0.get_camera_center();
  vgl_point_3d<T> a1 = p0.get_rotation()*p1.get_camera_center();
  vgl_point_3d<T> new_camera_center(a1.x() - a0.x(),
                                    a1.y() - a0.y(),
                                    a1.z() - a0.z() );
  new_camera.set_camera_center( new_camera_center );
  return new_camera;
}


//-------------------------------------------
template <class T>
vpgl_perspective_camera<T> vpgl_align_up(
  const vpgl_perspective_camera<T>& p0,
  const vpgl_perspective_camera<T>& p1 )
{
  vpgl_perspective_camera<T> new_camera;
  new_camera.set_calibration( p0.get_calibration() );
  new_camera.set_rotation( p1.get_rotation()*p0.get_rotation() );
  vgl_point_3d<T> a = p0.get_rotation().inverse()*p1.get_camera_center();
  vgl_point_3d<T> new_camera_center( p0.get_camera_center().x() + a.x(),
                                     p0.get_camera_center().y() + a.y(),
                                     p0.get_camera_center().z() + a.z() );
  new_camera.set_camera_center( new_camera_center );
  return new_camera;
}


//Post-multiply this perspective camera with a 3-d Euclidean transformation
// Must check if the transformation is Euclidean, i.e. rotation matrix
// and translation.   Since we can only work with the external interface
// the update due to the postmultiplication is:
//   K' = K
//   R' = R*Re
//   cc' = transpose(Re)(cc - te)
// where Re and te are the rotation matrix and
//  translation vector of the euclidean transform
template <class T> vpgl_perspective_camera<T>
vpgl_perspective_camera<T>::postmultiply( const vpgl_perspective_camera<T>& in_cam, const vgl_h_matrix_3d<T>& euclid_trans)
{
  assert(euclid_trans.is_euclidean());
  const vpgl_calibration_matrix<T>& K = in_cam.get_calibration();
  const vgl_rotation_3d<T>& R = in_cam.get_rotation();
  const vgl_point_3d<T>& cc = in_cam.get_camera_center();
  vgl_rotation_3d<T> Re(euclid_trans.get_upper_3x3());
  vgl_homg_point_3d<T> t = euclid_trans.get_translation();

  //The transformed rotation matrix
  vgl_rotation_3d<T> Rp(R*Re);

  //must have Euclidean quantities to proceed
  assert(!t.ideal());

  //Transform the camera center
  //get the Euclidean components
  vgl_vector_3d<T> te(t.x()/t.w(), t.y()/t.w(), t.z()/t.w());

  //construct the transformed center
  vgl_point_3d<T> ccp = Re.inverse()*(cc-te);

  return vpgl_perspective_camera<T>(K, ccp, Rp);
}

// I/O :------------------------------------------------

//: Write vpgl_perspective_camera to stream
template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s,
                         vpgl_perspective_camera<Type> const& p)
{
  vnl_matrix_fixed<Type, 3, 3> k = p.get_calibration().get_matrix();
  vgl_rotation_3d<Type> rot = p.get_rotation();
  vnl_matrix_fixed<Type, 3, 3> Rm = rot.as_matrix();
  vgl_vector_3d<Type> t = p.get_translation();
  s << k << '\n'
    << Rm << '\n'
    << t.x() << ' ' << t.y() << ' ' << t.z() << '\n';
  return s ;
}

//: Read camera from stream
template <class Type>
vcl_istream&  operator >>(vcl_istream& s,
                          vpgl_perspective_camera<Type>& p)
{
  vnl_matrix_fixed<Type, 3, 3> k, Rm;
  vnl_vector_fixed<Type, 3> tv;
  s >> k;
  s >> Rm;
  s >> tv;
  vpgl_calibration_matrix<Type> K(k);
  vgl_rotation_3d<Type> rot(Rm);
  vgl_vector_3d<Type> t(tv[0], tv[1], tv[2]);
  p.set_calibration(K);
  p.set_rotation(rot);
  p.set_translation(t);
  return s ;
}

//-------------------------------
template <class T> void vpgl_perspective_camera<T>::
b_read(vsl_b_istream &is)
{
  if (!is) return;

  vnl_matrix_fixed<T,4,4> Rot;
  vnl_vector_fixed<T,4> q;

  short ver;
  vsl_b_read(is, ver);
  switch (ver)
  {
   case 1:
     vpgl_proj_camera<T>::b_read(is);
     K_.b_read(is);
     vsl_b_read(is, camera_center_);
     vsl_b_read(is, Rot);
     R_ = vgl_rotation_3d<T>(vgl_h_matrix_3d<T>(Rot));
    break;
   case 2:
     vpgl_proj_camera<T>::b_read(is);
     K_.b_read(is);
     vsl_b_read(is, camera_center_);
     vsl_b_read(is, q);
     R_ = vgl_rotation_3d<T>(vnl_quaternion<T>(q));
    break;
   default:
    vcl_cerr << "I/O ERROR: vpgl_persperctive_camera::b_read(vsl_b_istream&)\n"
             << "           Unknown version number "<< ver << '\n';
    is.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
    return;
  }
  this->recompute_matrix();
}

//-------------------------------
//: Binary save self to stream.
// \remark cached_svd_ not written
template <class T> void vpgl_perspective_camera<T>::
b_write(vsl_b_ostream &os) const
{
  vsl_b_write(os, this->version());
  vpgl_proj_camera<T>::b_write(os);
  K_.b_write(os);
  vsl_b_write(os, camera_center_);
  vsl_b_write(os, static_cast<vnl_vector_fixed<T,4> >(R_.as_quaternion()));
}

//: Binary save
template <class T> void
vsl_b_write(vsl_b_ostream &os, const vpgl_perspective_camera<T> * p)
{
  if (p==0) {
    vsl_b_write(os, false); // Indicate null pointer stored
  }
  else{
    vsl_b_write(os,true); // Indicate non-null pointer stored
    p->b_write(os);
  }
}


//: Binary load
template <class T> void
vsl_b_read(vsl_b_istream &is, vpgl_perspective_camera<T>* &p)
{
  delete p;
  bool not_null_ptr;
  vsl_b_read(is, not_null_ptr);
  if (not_null_ptr) {
    p = new vpgl_perspective_camera<T>();
    p->b_read(is);
  }
  else
    p = 0;
}


// Code for easy instantiation.
#undef vpgl_PERSPECTIVE_CAMERA_INSTANTIATE
#define vpgl_PERSPECTIVE_CAMERA_INSTANTIATE(T) \
template class vpgl_perspective_camera<T >; \
template bool vpgl_perspective_decomposition(const vnl_matrix_fixed<T,3,4>& camera_matrix, \
                                             vpgl_perspective_camera<T >& p_camera ); \
template vpgl_perspective_camera<T > vpgl_align_down(const vpgl_perspective_camera<T >& p0, \
                                                     const vpgl_perspective_camera<T >& p1 ); \
template vpgl_perspective_camera<T > vpgl_align_up(const vpgl_perspective_camera<T >& p0, \
                                                   const vpgl_perspective_camera<T >& p1 ); \
template void vsl_b_read(vsl_b_istream &is, vpgl_perspective_camera<T >* &p); \
template void vsl_b_write(vsl_b_ostream &os, const vpgl_perspective_camera<T > * p); \
template vcl_ostream& operator<<(vcl_ostream&, const vpgl_perspective_camera<T >&); \
template vcl_istream& operator>>(vcl_istream&, vpgl_perspective_camera<T >&)

#endif // vpgl_perspective_camera_txx_
