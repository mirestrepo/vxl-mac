#include <rrel/rrel_muse_table.h>
#include <rrel/rrel_misc.h>
#include <vnl/vnl_math.h>

#include <vcl_cmath.h>
#include <vcl_cassert.h>
#include <vcl_iostream.h>

static const int max_allowed_to_store = 1000;

rrel_muse_table::rrel_muse_table( int max_n_stored )
{
  if ( max_n_stored > max_allowed_to_store ) {
    vcl_cerr << "WARNING: Requested rrel_muse_table size is larger than max allowed.\n"
             << "         Larger entries will be not be precomputed.\n";
    max_n_stored_ = max_allowed_to_store;
  }
  else
    max_n_stored_ = max_n_stored;

  expected_.resize( max_n_stored_+1, max_n_stored_+1 );
  standard_dev_.resize( max_n_stored_+1, max_n_stored_+1 );
  muse_t_divisor_.resize( max_n_stored_+1, max_n_stored_+1 );

  int k,n;
  for ( n=1; n<=max_n_stored_; ++n )
    for ( k=1; k<=n; ++k ) {
      expected_(k,n) = calculate_expected( k, n );
      standard_dev_(k,n) = calculate_standard_dev( k, n, expected_(k,n) );
      muse_t_divisor_(k,n) = calculate_divisor( k, n, expected_(k,n) );
    }
}


double
rrel_muse_table::expected_kth( int k, int n ) const
{
  assert( 0<k && k<= n );
  if ( n <= max_n_stored_ ) {
    return expected_(k,n);
  } else {
    return calculate_expected( k, n );
  }
}


double
rrel_muse_table::standard_dev_kth( int k, int n ) const
{
  assert( 0<k && k<=n );
  if ( n <= max_n_stored_ ) {
    return standard_dev_(k,n);
  } else {
    return calculate_standard_dev( k, n, calculate_expected(k,n) );
  }
}

double
rrel_muse_table::muset_divisor( int k, int n ) const
{
  assert( 0<k && k<= n );
  if ( n <= max_n_stored_ ) {
    return muse_t_divisor_(k,n);
  } else {
    return calculate_divisor( k, n, calculate_expected(k,n) );
  }
}


double
rrel_muse_table::calculate_expected( int k, int n ) const
{
  return rrel_misc_gaussian_cdf_inv(0.5*(1.0+((double)k / (double)(n+1))));
}


double
rrel_muse_table::calculate_standard_dev( int k, int n,
                                         double expected_kth ) const
{
  double pk, qk, Qk, pQk, Qk_prime, Qk_dprime, Qk_tprime, vrk;

  pk = (double) k / (double) (n+1); // might want alpha beta correction
  qk = 1.0 - pk;

  // calculate_ the inverse cdf (might want to an alpha-beta correction)
  Qk = expected_kth;  // ak(k, N);   // inverse cdf of absolute residuals

  // density of absolute residual evaluated at Qk
  pQk = vcl_exp( -0.5 * Qk*Qk) / vcl_sqrt(2.0*vnl_math::pi);

  // first derivative of Qk
  Qk_prime = 1.0/pQk;

  // second derivative of Qk
  Qk_dprime = Qk/(pQk*pQk);

  // third derivative of Qk
  Qk_tprime = ( 1.0 + 2.0 * Qk*Qk ) / (pQk*pQk*pQk);

  vrk = (pk*qk/(double)(n+2)) * Qk_prime*Qk_prime
        + (pk*qk/((double)((n+2)*(n+2)))) * ( 2.0*(qk - pk)*Qk_prime*Qk_dprime
        + pk*qk*(Qk_prime*Qk_tprime + 0.5*Qk_dprime*Qk_dprime));

  return vcl_sqrt(vrk);
}


double
rrel_muse_table::calculate_divisor( int k, int n,
                                    double expected_kth ) const
{
  return (n+1)*vcl_sqrt(2/vnl_math::pi)*(1.0-vcl_exp(-vnl_math_sqr(expected_kth)/2.0));
}

#if 0
void
rrel_muse_table_print( vcl_ostream& ostr,
                       const rrel_muse_table& table )
{
  ostr << "rrel_muse_table:\n";
  for( int n=2; n<=table.get_max_stored(); ++n ) {
    rrel_muse_table_print( ostr, table, n );
  }
}


void
rrel_muse_table_print( vcl_ostream& ostr,
                       const rrel_muse_table& table,
                       int n )
{
  for ( int k=1; k<n; ++k ) {
    ostr << k << " " << n << ":  " << table.expected_kth( k, n)
         << ", " << table.standard_dev_kth( k, n )
         << ", " << table.muset_divisor( k, n ) << "\n";
  }
}
#endif
