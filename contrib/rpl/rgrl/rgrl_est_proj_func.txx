#ifndef rgrl_est_proj_func_txx_
#define rgrl_est_proj_func_txx_

//:
// \file
// \author Gehua Yang
// \date   March 2007
// \brief  a generic class to estimate a homogeneous projection matrix using L-M

#include <rgrl/rgrl_est_proj_func.h>
#include <rgrl/rgrl_estimator.h>
#include <rgrl/rgrl_match_set.h>
#include <rgrl/rgrl_set_of.h>

#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_transpose.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/algo/vnl_svd.h>

#include <vcl_cassert.h>

namespace{

  template <unsigned int Tdim, unsigned int Fdim>
  inline
  void
  map_homo_point( vnl_vector_fixed<double, Tdim+1>& mapped,
                  vnl_matrix_fixed<double, Tdim+1, Fdim+1> const& proj,
                  vnl_vector_fixed<double, Fdim> const& loc )
  {
    for( unsigned i=0; i<Tdim+1; ++i ) {

      // shift term
      mapped[i] = proj(i, Fdim);

      for( unsigned j=0; j<Fdim; ++j )
        mapped[i] += loc[j] * proj(i,j);
    }
  }

  template <unsigned int Tdim, unsigned int Fdim>
  inline
  void
  map_inhomo_point( vnl_vector_fixed<double, Tdim>& mapped,
                    vnl_matrix_fixed<double, Tdim+1, Fdim+1> const& proj,
                    vnl_vector_fixed<double, Fdim> const& loc )
  {
    vnl_vector_fixed<double, Tdim+1> tmp;

    // map homo point
    for( unsigned i=0; i<Tdim+1; ++i ) {

      // shift term
      tmp[i] = proj(i, Fdim);

      for( unsigned j=0; j<Fdim; ++j )
        tmp[i] += loc[j] * proj(i,j);
    }

    // get inhomo point
    for( unsigned i=0; i<Tdim; ++i )
      mapped[i] = tmp[i]/tmp[Tdim];
  }
}

template <unsigned int Tdim, unsigned int Fdim>
rgrl_est_proj_func<Tdim, Fdim>::
rgrl_est_proj_func( rgrl_set_of<rgrl_match_set_sptr> const& matches,
                    bool with_grad )
: vnl_least_squares_function( (Fdim+1)*(Tdim+1)-1,
                              rgrl_est_matches_residual_number(matches),
                              with_grad ? use_gradient : no_gradient ),
  matches_ptr_( &matches ),
  from_centre_(double(0)), to_centre_(double(0)),
  max_num_iterations_(50),
  relative_threshold_(1e-8),
  zero_svd_thres_(1e-4)
{

}


//: convert parameters
template <unsigned int Tdim, unsigned int Fdim>
void
rgrl_est_proj_func<Tdim, Fdim>::
convert_parameters( vnl_vector<double>& params,
                    vnl_matrix_fixed<double, Tdim+1, Fdim+1>  proj_matrix,
                    vnl_vector_fixed<double, Fdim> const& fc,
                    vnl_vector_fixed<double, Tdim> const& tc )
{
  // set centres
  from_centre_ = fc;
  to_centre_ = tc;

  // make the prejection matrix to centre around from_centre_
  vnl_matrix_fixed<double, Fdim+1, Fdim+1> from_centre_matrix;
  from_centre_matrix.set_identity();
  for( unsigned i=0; i<Fdim; ++i )
    from_centre_matrix( i, Fdim ) = from_centre_[i];

  vnl_matrix_fixed<double, Tdim+1, Tdim+1> to_centre_matrix;
  to_centre_matrix.set_identity();
  for( unsigned i=0; i<Tdim; ++i )
    to_centre_matrix( i, Tdim ) = -to_centre_[i];

  proj_matrix = to_centre_matrix * proj_matrix * from_centre_matrix;

  // find out which element in the projection matrix has largest value
  double max_val = 0;
  index_row_ = index_col_ = -1;  // init to bad value
  for( unsigned i=0; i<Tdim+1; ++i )
    for( unsigned j=0; j<Fdim+1; ++j )
      if( vcl_abs( proj_matrix(i,j) ) > max_val ) {

        index_row_ = i;
        index_col_ = j;
        max_val = vcl_abs( proj_matrix(i,j) );
      }

  // normalize the proj_matrix to have the largest value as 1
  proj_matrix /= proj_matrix( index_row_, index_col_ );

  // fill in params
  params.set_size( (Fdim+1)*(Tdim+1) -1 );
  for( unsigned k=0,i=0; i<Tdim+1; ++i )
    for( unsigned j=0; j<Fdim+1; ++j ) {

      if( i==index_row_ && j==index_col_ ) {
        continue;
      }
      // fill in elements in order
      params[k++] = proj_matrix(i,j);
    }

}



template <unsigned int Tdim, unsigned int Fdim>
void
rgrl_est_proj_func<Tdim, Fdim>::
restored_centered_proj( vnl_matrix_fixed<double, Tdim+1, Fdim+1>& proj_matrix,
                        vnl_vector<double> const& params )
{
  assert( params.size() >= (Fdim+1)*(Tdim+1) - 1 );
  for( unsigned k=0,i=0; i<Tdim+1; ++i )
    for( unsigned j=0; j<Fdim+1; ++j ) {

      if( i==index_row_ && j==index_col_ ) {
        proj_matrix(i,j) = 1.0;
        continue;
      }
      // fill in elements in order
      proj_matrix(i,j) = params[k++];
    }
}

template <unsigned int Tdim, unsigned int Fdim>
void
rgrl_est_proj_func<Tdim, Fdim>::
f(vnl_vector<double> const& x, vnl_vector<double>& fx)
{
  typedef rgrl_match_set::const_from_iterator FIter;
  typedef FIter::to_iterator TIter;

  vnl_vector_fixed<double, Tdim> mapped;
  vnl_matrix_fixed<double, Tdim,Tdim> error_proj_sqrt;
  unsigned int ind = 0;

  // retrieve the projection matrix
  vnl_matrix_fixed<double, Tdim+1, Fdim+1> proj;
  restored_centered_proj( proj, x );

  for ( unsigned ms = 0; ms<matches_ptr_->size(); ++ms )
    if ( (*matches_ptr_)[ms] != 0 ) { // if pointer is valid

      rgrl_match_set const& one_set = *((*matches_ptr_)[ms]);
      for ( FIter fi=one_set.from_begin(); fi!=one_set.from_end(); ++fi ) {

        // map from point
        vnl_vector_fixed<double, Fdim> from = fi.from_feature()->location();
        from -= from_centre_;
        map_inhomo_point<Tdim, Fdim>( mapped, proj, from );

        for ( TIter ti=fi.begin(); ti!=fi.end(); ++ti ) {
          vnl_vector_fixed<double, Tdim> to = ti.to_feature()->location();
          to -= to_centre_;
          error_proj_sqrt = ti.to_feature()->error_projector_sqrt();
          double const wgt = vcl_sqrt(ti.cumulative_weight());
          vnl_vector_fixed<double, Tdim> diff = error_proj_sqrt * (mapped - to);

          // fill in
          for( unsigned i=0; i<Tdim; ++i )
            fx(ind++) = wgt * diff[i];
        }
      }
  }

  // check
  assert( ind == get_number_of_residuals() );
}

template <unsigned int Tdim, unsigned int Fdim>
void
rgrl_est_proj_func<Tdim, Fdim>::
gradf(vnl_vector<double> const& x, vnl_matrix<double>& jacobian)
{
  typedef rgrl_match_set::const_from_iterator FIter;
  typedef FIter::to_iterator TIter;

  const unsigned int param_num = (Fdim+1)*(Tdim+1);
  assert( jacobian.rows() == get_number_of_residuals() );
  assert( jacobian.cols() == param_num-1 );

  vnl_vector_fixed<double, Tdim+1> homo;
  vnl_matrix_fixed<double, Tdim,   Tdim>        error_proj_sqrt;
  vnl_matrix_fixed<double, Tdim,   param_num-1> base_jac, jac;
  vnl_matrix_fixed<double, Tdim,   param_num>   complete_jac;
  vnl_matrix_fixed<double, Tdim+1, param_num>   jf(0.0);    // grad in homogeneous coordinate
  vnl_matrix_fixed<double, Tdim,   Tdim+1>      jg(0.0); // grad of division, [u/w, v/w]^T

  // fill jf (linear gradient) with 1.0 on elements corresponding to shift
  for( unsigned i=0; i<Tdim+1; ++i )
    jf( i, i*(Fdim+1)+Fdim ) = 1.0;

  // retrieve the projection matrix
  vnl_matrix_fixed<double, Tdim+1, Fdim+1> proj;
  restored_centered_proj( proj, x );

  unsigned int ind = 0;
  for ( unsigned ms = 0; ms<matches_ptr_->size(); ++ms )
    if ( (*matches_ptr_)[ms] ) { // if pointer is valid

      rgrl_match_set const& one_set = *((*matches_ptr_)[ms]);
      for ( FIter fi=one_set.from_begin(); fi!=one_set.from_end(); ++fi ) {

        // map from point
        vnl_vector_fixed<double, Fdim> from = fi.from_feature()->location();
        from -= from_centre_;
        map_homo_point<Tdim, Fdim>( homo, proj, from );

        // linear gradient in homogeneous coordinate
        // skip the ones corresponding to shift
        for( unsigned index=0,i=0; i<Tdim+1; ++i,index+=(Fdim+1) )
          for( unsigned j=0; j<Fdim; ++j ) {
            jf( i, index+j ) = from[j];
          }

        // make division
        const double homo_last_neg_sqr = -vnl_math_sqr(homo[Tdim]);
        const double homo_last_div = 1.0/homo[Tdim];
        for( unsigned i=0; i<Tdim; ++i )
          jg(i,i) = homo_last_div;
        for( unsigned i=0; i<Tdim; ++i )
          jg(i,Tdim) = homo[i] / homo_last_neg_sqr;

        // since Jab_g(f(p)) = Jac_g * Jac_f
        complete_jac = jg * jf;

        // remove the element being held as constant 1
        const unsigned index = (index_row_*(Fdim+1)+index_col_);
        for( unsigned i=0,j=0; i<param_num; ++i )
          if( i != index ) {

            for( unsigned k=0; k<Tdim; ++k )
              base_jac(k, j) = complete_jac(k, i);

            ++j;
          }

        for ( TIter ti=fi.begin(); ti!=fi.end(); ++ti ) {
          //vnl_double_2 to = ti.to_feature()->location();
          error_proj_sqrt = ti.to_feature()->error_projector_sqrt();
          double const wgt = vcl_sqrt(ti.cumulative_weight());
          jac = error_proj_sqrt * base_jac;
          jac *= wgt;

          // fill in
          for( unsigned i=0; i<Tdim; ++i,++ind ) {

            for( unsigned j=0; j<param_num-1; ++j )
              jacobian(ind,j) = jac(i, j);
          }
        }
      }
  }
}

template <unsigned int Tdim, unsigned int Fdim>
bool
rgrl_est_proj_func<Tdim, Fdim>::
projective_estimate( vnl_matrix_fixed<double, Tdim+1, Fdim+1>& proj,
                     vnl_matrix<double>& full_covar,
                     vnl_vector_fixed<double, Fdim>& from_centre,
                     vnl_vector_fixed<double, Tdim>& to_centre )
{
  // compute weighted centres
  rgrl_est_compute_weighted_centres( *matches_ptr_,
                                     from_centre_.as_ref().non_const(),
                                     to_centre_.as_ref().non_const() );

  // convert parameters
  vnl_vector<double> p;
  this->convert_parameters( p, proj, from_centre, to_centre );

  vnl_levenberg_marquardt lm( *this );
  //lm.set_trace( true );
  //lm.set_check_derivatives( 1 );
  // we don't need it to be super accurate
  lm.set_f_tolerance( relative_threshold_ );
  lm.set_max_function_evals( max_num_iterations_ );

  bool ret;
  if ( has_gradient() )
    ret = lm.minimize_using_gradient(p);
  else
    ret = lm.minimize_without_gradient(p);
  if ( !ret ) {
    vcl_cerr <<  "Levenberg-Marquatt failed" << vcl_endl;
    return false;
  }
  //lm.diagnose_outcome(vcl_cout);

  // convert parameters back into matrix form
  this->restored_centered_proj( proj, p );

  // compute covariance
  // Jac^\top * Jac is INVERSE of the covariance
  //
  const unsigned int param_num = (Fdim+1)*(Tdim+1);
  vnl_matrix<double> jac(get_number_of_residuals(), param_num-1);
  this->gradf( p, jac );
  //

  // SVD decomposition:
  // Jac = U W V^\top
  vnl_svd<double> svd( jac, zero_svd_thres_ );
  if ( svd.rank() < param_num-1 ) {
    vcl_cerr <<  "The covariance of projection matrix ranks less than "
      << param_num-1 << "! " << vcl_endl ;
    return false;
  }

  //invert the W matrix and square it
  vnl_diag_matrix<double> invW( param_num-1 );
  for( unsigned i=0; i<param_num-1; ++i )
    invW[i] = vnl_math_sqr( 1.0/svd.W(i) );

  //compute inverse of Jac^\top * Jac
  const vnl_matrix<double>  covar( svd.V() * invW * vnl_transpose( svd.V() ) );

  // convert the covar to full dof+1 matrix
  // fill in the row and column of the fixed element
  // with 0
  full_covar.set_size( param_num, param_num );
  full_covar.fill( 0.0 );

  const unsigned int param_index = index_row_*(Fdim+1) + index_col_;
  for( unsigned i=0,ic=0; i<param_num; ++i ) {

    if( i==param_index ) continue;

    for( unsigned j=0,jc=0; j<param_num; ++j ) {

      if( j==param_index ) continue;
      full_covar( i, j ) = covar( ic, jc );
      ++jc;
    }

    //increment the index count in covar matrix
    ++ic;
  }

  return true;
}


// // --------------------------------------------------------------------
// template <unsigned int Tdim, unsigned int Fdim>
// rgrl_est_proj_lm::
// rgrl_est_proj_lm( bool with_grad )
//   : with_grad_( with_grad )
// {
//    rgrl_estimator::set_param_dof( Fdim*Tdim-1 );
//
//   // default value
//   rgrl_nonlinear_estimator::set_max_num_iter( 50 );
//   rgrl_nonlinear_estimator::set_rel_thres( 1e-5 );
// }
//
// template <unsigned int Tdim, unsigned int Fdim>
// bool
// rgrl_est_proj_lm::
// projective_estimate( vnl_matrix_fixed<double, Tdim+1, Fdim+1>& proj,
//                      vnl_matrix<double>& full_covar,
//                      vnl_vector_fixed<double, Fdim>& from_centre,
//                      vnl_vector_fixed<double, Tdim>& to_centre,
//                      rgrl_set_of<rgrl_match_set_sptr> const& matches ) const
// {
//   // count the number of constraints/residuals
//   typedef rgrl_match_set::const_from_iterator FIter;
//   typedef FIter::to_iterator TIter;
//   unsigned int tot_num = 0;
//   for ( unsigned ms = 0; ms<matches.size(); ++ms )
//     if ( matches[ms] ) { // if pointer is valid
//
//       rgrl_match_set const& one_set = *(matches[ms]);
//       for ( FIter fi=one_set.from_begin(); fi!=one_set.from_end(); ++fi )
//         if( fi.size() ) {
//           tot_num += fi.size() * fi.begin().to_feature()->dim();  // each point provides two constraints
//         }
//     }
//
//   // Determine the weighted centres for computing the more stable
//   // covariance matrix of homography parameters
//   //
//   if( !compute_weighted_centres( matches, from_centre, to_centre ) )
//     return 0;
//    DebugMacro( 3, "From center: " << from_centre
//                <<"  To center: " << to_centre << vcl_endl );
//
//   // construct least square cost function
//   rgrl_est_proj_func<Tdim, Fdim> proj_func( matches, tot_num, with_grad_ );
//
//   // convert parameters
//   vnl_vector<double> p;
//   proj_func.convert_parameters( p, proj, from_centre, to_centre );
//
//   vnl_levenberg_marquardt lm( proj_func );
//   lm.set_trace( true );
//   lm.set_check_derivatives( 1 );
//   // we don't need it to be super accurate
//   lm.set_f_tolerance( relative_threshold_ );
//   lm.set_max_function_evals( max_num_iterations_ );
//
//   bool ret;
//   if ( with_grad_ )
//     ret = lm.minimize_using_gradient(p);
//   else
//     ret = lm.minimize_without_gradient(p);
//   if ( !ret ) {
//     WarningMacro( "Levenberg-Marquatt failed" );
//     return 0;
//   }
//   lm.diagnose_outcome(vcl_cout);
//
//   // convert parameters back into matrix form
//   restored_centered_proj( proj, p );
//
//   // compute covariance
//   // Jac^\top * Jac is INVERSE of the covariance
//   //
//   const unsigned int param_num = (Fdim+1)*(Tdim+1);
//   vnl_matrix<double> jac(tot_num, param_num-1);
//   proj_func.gradf( p, jac );
//   //
//
//   // SVD decomposition:
//   // Jac = U W V^\top
//   vnl_svd<double> svd( jac, 1e-4 );
//   if ( svd.rank() < param_num-1 ) {
//     WarningMacro( "The covariance of homography ranks less than 8! ");
//     return 0;
//   }
//
//   //invert the W matrix and square it
//   vnl_diag_matrix<double> invW( param_num-1 );
//   for( unsigned i=0; i<param_num-1; ++i )
//     invW[i] = vnl_math_sqr( 1.0/svd.W(i) );
//
//   //compute inverse of Jac^\top * Jac
//   const vnl_matrix<double>  covar( svd.V() * invW * vnl_transpose( svd.V() ) );
//
//   // convert the covar to full dof+1 matrix
//   // fill in the row and column of the fixed element
//   // with 0
//   full_covar.set_size( param_num, param_num );
//
//   const unsigned int param_index = index_row_*(Fdim+1) + index_col_;
//   for( unsigned i=0,ic=0; i<param_num; ++i ) {
//
//     if( i==param_index ) continue;
//
//     for( unsigned j=0;jc=0; j<param_num; ++j ) {
//
//       if( j==param_index ) continue;
//       full_covar( i, j ) = covar( ic, jc );
//       ++jc;
//     }
//
//     //increment the index count in covar matrix
//     ++ic;
//   }
// }

#undef  RGRL_EST_PROJ_FUNC_INSTANTIATE
#define RGRL_EST_PROJ_FUNC_INSTANTIATE( tdim, fdim ) \
  template class rgrl_est_proj_func< tdim, fdim >

#endif //rgrl_est_proj_func_txx_
