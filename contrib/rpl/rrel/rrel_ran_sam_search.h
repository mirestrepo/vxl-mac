#ifndef rrel_ran_sam_search_h_
#define rrel_ran_sam_search_h_

//:
// \file
// \author Chuck Stewart (stewart@cs.rpi.edu)
// \date March 2001
// Random sampling search for minimization of a robust objective function.

#include <vnl/vnl_vector.h>
#include <mbl/mbl_mz_random.h>
#include <vcl_vector.h>

class rrel_objective;
class rrel_estimation_problem;


//: Random sampling search for minimization of a robust objective function.
//  The code organization follows the design of the rrel
//  library where there is a separation between objective function
//  (M-estimator, LMS, etc), search technique, and estimation problem.
//  See discussion in rrel_estimation_base.
//  
//  The algorithm implemented was discovered independently by Fischler
//  and Bolles in the CACM RANSAC paper from 1981 and Rousseeuw in a
//  Journal of the American Statistical Association paper from 1984.
//  The implementation follows Rousseeuw's approach by not allowing
//  the short-circuiting of RANSAC, which can cause incorrect results
//  for a large number of outliers.
//
//  The number of samples taken is computed to accomodate the
//  possibility of more than one population in the data, as described
//  in Stewart, PAMI 1995.  It is a relatively straightforward
//  generalization of the ordinary method for calculating the number
//  of samples.
//
//  Using this class requires calling set_sampling_params (or
//  set_gen_all_samples, which can be quite expensive), and then
//  calling estimate().  Results may be obtained through the functions
//  params() and scale().

class rrel_ran_sam_search {
public:
  //: Constructor using a non-deterministic random-sampling seed.
  rrel_ran_sam_search( );

  //: Constructor using a given random-sampling seed.
  rrel_ran_sam_search( int seed );

  ~rrel_ran_sam_search( ) {}

  //  Parameters to control the search technique.  The default set
  //  when the constructor is called is to sample as in generate
  //  samples as specified in least-median of squares.

  //: Indicate that all possible minimal subset samples should be tried.
  void set_gen_all_samples();

  //: Set the parameters for random sampling.
  void set_sampling_params( double max_outlier_frac = 0.5,
                            double desired_prob_good = 0.99,
                            unsigned int max_populations_expected = 1,
                            unsigned int min_samples = 0 );

  // ----------------------------------------
  //  Main estimation functions
  // ----------------------------------------

  //: \brief Estimation for an "ordinary" estimation problem.
  bool estimate( const rrel_estimation_problem* problem,
                 const rrel_objective* obj_fcn );

  // -----------------------------------------
  // Access to results and computed parameters
  // -----------------------------------------

  //:  Get the scale estimate.
  double scale() const { return scale_; }
  
  //:  Get the parameter estimate.
  const vnl_vector<double>& params() const { return params_; }

  //:  Get the number of samples tested in during estimation.
  int samples_tested() const { return samples_to_take_; }

  //:  Print the sampling parameters.  Used for debugging.
  void print_params() const;

  void set_trace_level( int level ) { trace_level_ = level; }

private:
  // ------------------------------------------------------------
  //  Random sampling functions.  Don't call directly.  These are
  //  public for test purposes.
  // ------------------------------------------------------------
  
  //: Calculate number of samples --- non-unique matching estimation problems 
  void calc_num_samples( const rrel_estimation_problem* problem );
  
  //: Determine the next random sample, filling in the "sample" vector.
  void next_sample( unsigned int taken, unsigned int num_points, vcl_vector<int>& sample,
                    unsigned int points_per_sample );

private:

  void trace_sample( const vcl_vector<int>& point_indices ) const;
  void trace_residuals( const vcl_vector<double>& residuals ) const;

protected:
  //
  //  Parameters
  //
  double max_outlier_frac_;
  double desired_prob_good_;
  unsigned int max_populations_expected_;
  unsigned int min_samples_;
  bool generate_all_;

  //
  // Random number generator
  //
  mbl_mz_random generator_;

  //
  //  The estimate
  //
  vnl_vector<double> params_;
  double scale_;

  //
  //  Sampling variables
  //
  unsigned int samples_to_take_;

  int trace_level_;
};

#endif
