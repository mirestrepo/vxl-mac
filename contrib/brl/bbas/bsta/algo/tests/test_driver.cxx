#include <testlib/testlib_register.h>

DECLARE( test_fit_weibull );
DECLARE( test_gaussian_model );
DECLARE( test_mean_shift );

void
register_tests()
{
  REGISTER( test_fit_weibull );
  REGISTER( test_gaussian_model );
  REGISTER( test_mean_shift );
}

DEFINE_MAIN;
