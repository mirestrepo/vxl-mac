#include <vcl_iostream.h>
#include <vcl_cmath.h>
#include <vcl_vector.h>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

#include <rrel/rrel_tukey_obj.h>
#include <rrel/rrel_linear_regression.h>
#include <rrel/rrel_irls.h>

#include <testlib/testlib_test.h>

#include "similarity_from_matches.h"

bool close(double,double);

double noise( double sigma );

const double conv_tolerance=1.0e-5;

void
regression_points( const vnl_vector<double>& a,
                   double sigma,
                   vcl_vector< vnl_vector<double> >& pts )
{
  const int num_pts=20;
  pts.resize( num_pts );
  double x, y, z;
  vnl_vector<double> p(3);

  //  Initialize variables.
  x = 1.0; y=-0.5; z= a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[0]=p;

  x = 2.0;  y=4.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[1]=p;

  x = 3.0;  y=1.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[2]=p;

  x = -2.0;  y=3.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[3]=p;

  x = 2.0;  y=4.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[4]=p;

  x = 5.0;  y=-4.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[5]=p;

  x = 3.0;  y=-2.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[6]=p;

  x = 2.0;  y=-2.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[7]=p;

  x = 3.0;  y=0.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[8]=p;

  x = -1.0; y=-2.0; z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[9]=p;

  x = 0.0;  y=0.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[10]=p;

  x = -1.0; y=2.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[11]=p;

  x = 3.5; y=5.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[12]=p;

  x = 4.5; y=5.5;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[13]=p;

  x = 3.5; y=6.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[14]=p;

  x = -3.5; y=5.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[15]=p;

  x = -2.5; y=-4.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[16]=p;

  x = 3.5; y=7.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[17]=p;

  x = -4.5; y=-4.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[18]=p;

  x = 6.5; y=4.0;  z = a[0] + a[1]*x + a[2]*y;
  p[0] = x; p[1]=y; p[2]=z;  pts[19]=p;

  for ( int i=0; i<num_pts; ++i )
    if ( i%10 == 5 )  //
      pts[i].z() += noise( 20 * sigma);
    else
      pts[i].z() += noise( sigma );
}

bool
check( const vnl_vector<double>& correct_params,
       rrel_irls* irls )
{
  vnl_vector<double> res(irls->params());
  double s = irls->scale();
  vnl_matrix<double> covar(irls->cofactor()*s*s);
  vnl_vector<double> err_vector(res - correct_params);

  //  Get standardized error
  double err = vcl_sqrt(dot_product( err_vector*covar, err_vector ));
#ifdef DEBUG
  vcl_cerr << "Mahalanobis param error = " << err << ", scale = " << s << "\n";
#endif
  bool success = err < 0.5*s;
#if 0
  bool conv = ( irls->did_it_converge( ) );
  vcl_cout << "Finished:\n"
           << "  estimate = " << irls->estimate()
           << ", true fit = " << correct_params << "\n"
           << "  Mahalanobis param error = " << err << "\n"
           << (success ? "success" : "fail") << "\n"
           << "  scale estimate = " << s  << "\n"
           << "  iterations used = " << irls->iterations_used() << "\n"
           << "  did it converge? " << (conv ? "yes\n" : "no\n");
#endif
 return success;
}


MAIN( test_irls )
{
  START( "rrel_irls" );

  //  Set true parameter estimate.
  double a[] = { 10.0, 0.02, -0.1 };
  vnl_vector<double> true_params(3, 3, a);

  //  Create the linear regression problem and an m_estimator objective
  //  function.
  //
  bool use_intercept=true;
  double sigma = 0.25;
  vcl_vector< vnl_vector<double> > pts;
  regression_points( true_params, sigma, pts );
  rrel_linear_regression * lr = new rrel_linear_regression( pts, use_intercept );
  int dof = lr->param_dof();
  rrel_wls_obj * m_est = new rrel_tukey_obj( dof );
  int max_iterations = 50;
  int trace_level=0;
  testlib_test_begin( "ctor" );
  rrel_irls * irls = new rrel_irls( max_iterations );
  testlib_test_perform( irls != 0 );

  //  Setting max iteration parameters.
  max_iterations = 15;
  irls->set_max_iterations( max_iterations );

  //  Setting scale estimation parameters.
  int iterations_for_scale = 2;
  bool use_weighted_scale = false;
  testlib_test_begin( "scale parameters for non-weighted scale" );
  irls->set_est_scale( iterations_for_scale, use_weighted_scale );
  testlib_test_perform( true );
  testlib_test_begin( "use convergence test" );
  irls->set_convergence_test( conv_tolerance );
  testlib_test_perform( true );

  testlib_test_begin( "irls with scale estimation" );
  bool success = irls->estimate( lr, m_est ) && check( true_params, irls );
  vcl_cout << "scale:  correct = " << sigma << ", estimated = " << irls->scale() << vcl_endl;
  testlib_test_perform( success );

  irls->reset_params();
  irls->reset_scale();
  use_weighted_scale = true;
  irls->set_est_scale( iterations_for_scale, use_weighted_scale );

  testlib_test_begin( "irls with weighted scale" );
  success = irls->estimate( lr, m_est ) && check( true_params, irls );
  vcl_cout << "scale:  correct = " << sigma << ", estimated = " << irls->scale() << vcl_endl;
  testlib_test_perform( success );
  testlib_test_begin( "did it converge?" );
  testlib_test_perform( irls->converged() );

  irls->reset_params();
  irls->reset_scale();
  irls->initialize_params( true_params );
  testlib_test_begin( "irls with correct initial fit" );
  success = irls->estimate( lr, m_est ) && check( true_params, irls );
  vcl_cout << "scale:  correct = " << sigma << ", estimated = " << irls->scale() << vcl_endl;
  testlib_test_perform( success );

  irls->reset_params();
  irls->initialize_scale( sigma );
  irls->set_no_scale_est();
  irls->set_no_convergence_test();

  testlib_test_begin( "irls with fixed scale" );
  max_iterations=6;
  irls->set_max_iterations( max_iterations );
  success = irls->estimate( lr, m_est ) && check( true_params, irls );
  testlib_test_perform( success );
  testlib_test_begin( "scale unchanged" );
  testlib_test_perform( irls->scale() == sigma );
  testlib_test_begin( "iterations used" );
  testlib_test_perform( irls->iterations_used() == max_iterations );


  //  onto irls from matches
  trace_level = 0;
  vnl_vector<double> params(4);
  params[0] = 1.2;  params[1] = 0.3;  params[2] = 15;  params[3] = -4;
  vcl_vector< image_point_match > matches;
  sigma = 0.25;
  generate_similarity_matches( params, sigma, matches );
  rrel_estimation_problem* match_prob = new similarity_from_matches( matches );

  rrel_irls irls_m( 20 );
  irls_m.set_trace_level( trace_level );
  irls_m.set_est_scale( 2, true );  // use weighted
  irls_m.set_convergence_test();
  irls_m.initialize_params( params );

  testlib_test_begin( "non-unique matches -- params initialized correctly, weighted scale" );
  testlib_test_perform( irls_m.estimate( match_prob, m_est ) &&
                    check( params, &irls_m ) );
  vcl_cout << "true scale = " << sigma << ", weighted scale = " << irls_m.scale() << vcl_endl;

  irls_m.reset_params();
  irls_m.initialize_params( params );
  irls_m.set_est_scale( 2, false );  // use un-weighted scale
  irls_m.set_convergence_test();
  testlib_test_begin( "non-unique matches -- params initialized correctly, MAD scale" );
  testlib_test_perform( irls_m.estimate( match_prob, m_est ) &&
                    check( params, &irls_m ) );
  vcl_cout << "true scale = " << sigma << ", MAD scale = " << irls_m.scale() << vcl_endl;

  irls_m.reset_params();
  irls_m.initialize_params( params );
  irls_m.set_no_scale_est( );  // use no scale
  irls_m.initialize_scale( sigma );
  irls_m.set_convergence_test();
  testlib_test_begin( "non-unique matches -- params initialized correctly, fixed scale" );
  testlib_test_perform( irls_m.estimate( match_prob, m_est ) &&
                    check( params, &irls_m ) );
  testlib_test_begin( "scale unchanged" );
  testlib_test_perform( close( sigma, irls_m.scale() ) );

  SUMMARY();
}
