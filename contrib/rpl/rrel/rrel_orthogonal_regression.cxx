#include <rrel/rrel_orthogonal_regression.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vnl/algo/vnl_svd.h>

#include <vcl_iostream.h>
#include <vcl_vector.h>

rrel_orthogonal_regression::rrel_orthogonal_regression( const vnl_matrix<double>& pts )
  : vars_( pts )
{
  unsigned int num_pts = pts.rows ();
  set_param_dof( pts.cols()-1 ); // up to a scale
  if ( param_dof() > num_pts ) {
    vcl_cerr << "\nrrel_orthogonal_regression::rrel_orthogonal_regression  WARNING:  DoF is greater than\n"
         << "the number of data points.  An infinite set of equally valid\n"
         << "solutions exists.\n";
  }
  set_num_samples_for_fit( param_dof() );
}  

rrel_orthogonal_regression::rrel_orthogonal_regression( const vcl_vector<vnl_vector<double> >& pts )
  : vars_( pts.size(),pts[0].size() )
{
  for (unsigned int i=0;i<vars_.rows();i++)
    for (unsigned int j=0;j<vars_.cols();j++)
      vars_ (i,j) = pts [i][j];
  
  unsigned int num_pts = vars_.rows();
  set_param_dof( vars_.cols()-1 ); // up to a scale
  if ( param_dof() > num_pts ) {
    vcl_cerr << "\nrrel_orthogonal_regression::rrel_orthogonal_regression  WARNING:  DoF is greater than\n"
         << "the number of data points.  An infinite set of equally valid\n"
         << "solutions exists.\n";
  }
  set_num_samples_for_fit( param_dof() );
}

rrel_orthogonal_regression::~rrel_orthogonal_regression()
{
}

unsigned int
rrel_orthogonal_regression::num_samples( ) const
{
  return vars_.rows();
}


bool
rrel_orthogonal_regression::fit_from_minimal_set( const vcl_vector<int>& point_indices,
                                                  vnl_vector<double>& params ) const
{
  if ( point_indices.size() != param_dof() ) {
    vcl_cerr << "rrel_orthogonal_regression::fit_from_minimal_sample  The number of point "
         << "indices must agree with the fit degrees of freedom.\n";
    return false;
  }

  // The equation to be solved is A p = 0, where A is a dof_ x (dof_+1)
  // because the solution is up to a scale.
  // We solve the uniqueness by adding another constraint ||p||=1

  vnl_matrix<double> A(param_dof(), param_dof()+1);
  for ( unsigned int i=0; i<param_dof(); ++i ) {
    int index = point_indices[i];
    for ( unsigned int j=0; j<param_dof()+1; ++j ) {
      A(i,j) = vars_(index,j);
    }
  }

  vnl_svd<double> svd( A, 1.0e-8 );
  if ( (unsigned int)svd.rank() < param_dof() ) {
  	vcl_cerr << "rrel_orthogonal_regression:: singular fit!\n";
    return false;    // singular fit 
  }
  else {
    params = svd.nullvector();
    return true;
  }
}

void 
rrel_orthogonal_regression::compute_residuals( const vnl_vector<double>& params,
                                               vcl_vector<double>& residuals ) const
{
  // The residual is the algebraic distance, which is simply A * p.
  if ( residuals.size() != vars_.rows())
    residuals.resize( vars_.rows() );
  for ( unsigned int i=0; i<vars_.rows(); ++i ) {
    residuals[i] = dot_product( params, vars_.get_row(i) );
  }
}


// Compute a least-squares fit, using the weights if they are provided.
// The cofact matrix is not used or set.
bool
rrel_orthogonal_regression::weighted_least_squares_fit( vnl_vector<double>& params,
                                                        vnl_matrix<double>& cofact,
                                                        vcl_vector<double> *weights ) const
{
  // If params and cofact are NULL pointers and the fit is successful,
  // this function will allocate a new vector and a new
  // matrix. Otherwise, it assumes that *params is a param_dof() x 1 vector
  // and that cofact is param_dof() x param_dof() matrix.

  assert( !weights || weights->size() == vars_.rows() );

  vnl_matrix<double> A(vars_.rows(), vars_.cols());

  if (weights)
    for ( unsigned int i=0; i<vars_.rows(); ++i )
      for ( unsigned int j=0; j<vars_.cols(); ++j ) {
        A(i,j) = vars_(i,j) * (*weights)[i];
      }
  
  else A = vars_;

  vnl_svd<double> svd( A, 1.0e-8 );
  if ( (unsigned int)svd.rank() < param_dof() ) {
    vcl_cerr << "rrel_orthogonal_regression::WeightedLeastSquaresFit --- singularity!\n";
    return false;
  }
  else {
    params = svd.nullvector();
    return true;
  }
}


void
rrel_orthogonal_regression::print_points() const
{
  vcl_cout << "\nrrel_orthogonal_regression::print_points:\n"
           << "  param_dof() = " << param_dof() << "\n"
           << "  num_pts = " << vars_.rows() << "\n\n"
           << " i   vars_ \n"
           << " =   ========= \n";
  for ( unsigned int i=0; i<vars_.rows(); ++i ) {
    vcl_cout << " " << i << "   " << vars_.get_row (i) << "\n";
  }
}
