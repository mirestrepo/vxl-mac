#ifndef bugl_normal_point_2d_h_
#define bugl_normal_point_2d_h_

#include <vbl/vbl_ref_count.h>
#include <bugl/bugl_gaussian_point_2d.h>

//: \brief a specification of template class bugl_gaussian_point_2d
//
class bugl_normal_point_2d: public vbl_ref_count
{
  bugl_gaussian_point_2d<double> p_;
  
  public:
  bugl_normal_point_2d(vgl_point_2d<double> &p, vnl_matrix_fixed<double, 2, 2> &s) : p_(p, s) {}
  bugl_normal_point_2d(double x, double y, vnl_matrix_fixed<double, 2, 2> &s) : p_(x, y, s) {}
  bugl_normal_point_2d();

  virtual ~bugl_normal_point_2d() {}

 //: get covariant matrix
  vnl_matrix_fixed<double, 2, 2> get_covariant_matrix() const 
  { return p_.get_covariant_matrix();}

 //: set covariant matrix
 void set_covariant_matrix(vnl_matrix_fixed<double, 2, 2>& s) {  p_.set_covariant_matrix(s);}
 
 //: set point value
 void set_point(vgl_point_2d<double> & p) {p_.set_point(p);}
 
 //: probability density at point p
 double  prob_at(vgl_point_2d<double>  &p) { return p_.prob_at(p); }

};

#endif
