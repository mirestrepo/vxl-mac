//:
// \file
// \brief Program demonstrating use the Robust Estimation library in line fitting
//
// \author Chuck Stewart
// \author Bess Lee
// Modifications: Oct 2001 Amitha Perera: added comments.

#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>

#include <rrel/rrel_linear_regression.h>
#include <rrel/rrel_lms_obj.h>
#include <rrel/rrel_ran_sam_search.h>
#include <rrel/rrel_irls.h>
#include <rrel/rrel_ransac_obj.h>
#include <rrel/rrel_trunc_quad_obj.h>
#include <rrel/rrel_m_est_obj.h>
#include <rrel/rrel_tukey_obj.h>
#include <rrel/rrel_muset_obj.h>
#include <rrel/rrel_muse_table.h>

int
main( )
{
  //
  // Read in the data points.
  //
  // The files line_fit_30.dat and line_fit_60.dat contain data from a
  // line corrupted by small scale noise, and with 30% and 60% large
  // scale noise, respectively. There is also a small step
  // discontinuty on the right edge. The points were generated by
  // line_gen.
  //
  vcl_vector< vnl_vector<double> > pts;
  vnl_vector<double> p(2);
  double x, y;

  while ( vcl_cin >> x >> y ) {
    p[0] = x; p[1] = y;
    pts.push_back(p);
  }

  // We wish to fit a line to the (x,y) points, assuming x is the
  // independent variable and y is the dependent variable. This is a
  // linear regression problem. We wish to estimate y = ax + b, but
  // the data is of the form (x,y), not (1,x,y), so we set
  // use_intercept=true.
  //
  bool use_intercept=true;
  rrel_linear_regression * lr = new rrel_linear_regression( pts, use_intercept );

  // This controls the verbosity of the search techniques.
  int trace_level = 0;

  // These next three parameters are used in the random sampling
  // searches, not in the IRLS searches.

  // The maximum fraction of the data that is expected to be gross outliers.
  double max_outlier_frac = 0.5;

  // The desired probability of finding the correct fit.
  double desired_prob_good = 0.99;

  // The number of different populations in the data set. For most
  // problems, the data is from one source (surface, etc.), so this
  // will be 1.
  int max_pops = 1;

  // Now we try different objective function/search technique
  // combinations to solve this linear regression problem.

  //
  // Pure least-squares.
  //
  // Most problems implement the weighted_least_squares_fit()
  // function, which means that the IRLS search technique can be used
  // on those problems. It also means that we can do a simple LS by
  // calling the function directly without providing a weight vector.
  //
  {
    vnl_vector<double> ls_params;
    vnl_matrix<double> ls_norm_covar;
    if ( !lr->weighted_least_squares_fit( ls_params, ls_norm_covar ) ) {
      vcl_cout << "Regression failed!!\n";
    }
    else {
      vcl_cout << "Regression succeeded.\n";
      vcl_cout << "estimate = " << ls_params[0] << " + " << ls_params[1] << " * x" << vcl_endl;
    }
    vcl_cout << vcl_endl;
  }

  //
  //  Least Median of Squares
  //
  {
    int num_sam_inst = lr->num_samples_to_instantiate();
    rrel_objective* lms = new rrel_lms_obj( num_sam_inst );
    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    ransam->set_trace_level(trace_level);

    if ( !ransam->estimate( lr, lms) )
      vcl_cout << "LMS failed!!\n";
    else {
      vcl_cout << "LMS succeeded.\n";
      vcl_cout << "estimate = " << ransam->params()[0] << " + " << ransam->params()[1] << " * x" << vcl_endl
               << "scale = " << ransam->scale() << vcl_endl;
    }
    vcl_cout << vcl_endl;

    delete ransam;
    delete lms;
  }

  //
  //  RANSAC
  //
  {
    rrel_ransac_obj* ransac = new rrel_ransac_obj( );
    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    ransam->set_trace_level(trace_level);
    lr->set_prior_scale(1.0);

    if ( !ransam->estimate( lr, ransac) )
      vcl_cout << "RANSAC failed!!\n";
    else {
      vcl_cout << "RANSAC succeeded.\n";
      vcl_cout << "estimate = " << ransam->params()[0] << " + " << ransam->params()[1] << " * x" << vcl_endl
               << "scale = " << ransam->scale() << vcl_endl;
    }
    vcl_cout << vcl_endl;

    delete ransac;
    delete ransam;
  }

  //
  //  MSAC
  //
  {
    rrel_trunc_quad_obj* msac = new rrel_trunc_quad_obj( );
    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    ransam->set_trace_level(trace_level);
    lr->set_prior_scale(1.0);

    if ( !ransam->estimate( lr, msac) )
      vcl_cout << "MSAC failed!!\n";
    else {
      vcl_cout << "MSAC succeeded.\n";
      vcl_cout << "estimate = " << ransam->params()[0] << " + " << ransam->params()[1] << " * x" << vcl_endl
               << "scale = " << ransam->scale() << vcl_endl;
    }
    vcl_cout << vcl_endl;

    delete msac;
    delete ransam;
  }

  //
  //  MUSE
  //
  {
    rrel_muset_obj* muset = new rrel_muset_obj( pts.size()+1 );
    muset -> set_min_inlier_fraction( 0.25 );

    lr->set_no_prior_scale();

    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_sampling_params( 1 - muset->min_inlier_fraction(),
                                 desired_prob_good,
                                 max_pops);
    ransam->set_trace_level(trace_level);

    if ( !ransam->estimate( lr, muset) )
      vcl_cout << "MUSE failed!!\n";
    else {
      vcl_cout << "MUSE succeeded.\n";
      vcl_cout << "estimate = " << ransam->params()[0] << " + " << ransam->params()[1] << " * x" << vcl_endl
               << "scale = " << ransam->scale() << vcl_endl;
    }
    vcl_cout << vcl_endl;

    delete muset;
    delete ransam;
  }


  //
  //  IRLS implementation of an M-estimator. (Beaton-Tukey loss
  //  function.)
  //
  //  IRLS needs an initial parameter estimate to begin the
  //  iterations. If none is given, as in this case, it will use a
  //  standard LS to initialise. This can often lead to disaster,
  //  especially in the presence many gross outliers. In general, you
  //  should provide an initial parameter estimate.
  //
  {
    rrel_m_est_obj * m_est = new rrel_tukey_obj( 4.0 );

    lr->set_no_prior_scale();

    // With these parameters, the IRLS will run for a maximum of 15
    // iterations, and a minimum of 2. During the first two
    // iterations, scale will be estimated; this estimate will be used
    // from the third iteration on. The iterations will also stop if
    // the objective functions converges (either absolutely or
    // relatively).

    int max_iterations = 15;
    int iterations_for_scale = 2;
    double conv_tolerance = 1e-4;

    rrel_irls* irls = new rrel_irls( max_iterations );
    irls->set_est_scale( iterations_for_scale );
    irls->set_convergence_test( conv_tolerance );
    irls->set_trace_level(trace_level);

    if ( !irls->estimate( lr, m_est ) ) {
      vcl_cout << "M-est (IRLS) failed!\n";
    }
    else {
      vcl_cout << "M-est (IRLS) succeeded.\n";
      vcl_cout << "estimate = " << irls->params()[0] << " + " << irls->params()[1] << " * x" << vcl_endl
               << "scale = " << irls->scale() << vcl_endl;
    }

    delete irls;
    delete m_est;
  }

  {
    rrel_m_est_obj* m_est = new rrel_tukey_obj( 4.0 );

    lr->set_no_prior_scale();

    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    ransam->set_trace_level(trace_level);

    if ( !ransam->estimate( lr, m_est) )
      vcl_cout << "M-est (RANSAM) failed!!\n";
    else {
      vcl_cout << "M-est (RANSAM) succeeded.\n";
      vcl_cout << "estimate = " << ransam->params()[0] << " + " << ransam->params()[1] << " * x" << vcl_endl
               << "scale = " << ransam->scale() << vcl_endl;
    }
    vcl_cout << vcl_endl;

    delete m_est;
    delete ransam;
  }

  delete lr;

  return 0;
}
