#include <testlib/testlib_register.h>

DECLARE( test_image_view );
DECLARE( test_math );
DECLARE( test_trilin_interp );
DECLARE( test_sample_profile_trilin );
DECLARE( test_gauss_reduce );
DECLARE( test_switch_axes );
DECLARE( test_algo_threshold );
DECLARE( test_algo_structuring_element );
DECLARE( test_algo_binary_dilate );
DECLARE( test_algo_binary_erode );
DECLARE( test_algo_exp_filter );
DECLARE( test_algo_grad_1x3 );

void
register_tests()
{
  REGISTER( test_image_view );
  REGISTER( test_math );
  REGISTER( test_trilin_interp );
  REGISTER( test_sample_profile_trilin );
  REGISTER( test_gauss_reduce );
  REGISTER( test_switch_axes );
  REGISTER( test_algo_threshold );
  REGISTER( test_algo_structuring_element );
  REGISTER( test_algo_binary_dilate );
  REGISTER( test_algo_binary_erode );
  REGISTER( test_algo_exp_filter );
  REGISTER( test_algo_grad_1x3 );
}

DEFINE_MAIN;
