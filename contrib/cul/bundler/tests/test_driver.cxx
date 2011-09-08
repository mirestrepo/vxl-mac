#include <testlib/testlib_register.h>

DECLARE( test_bundler_utils );
DECLARE( test_tracks_detect );
DECLARE( test_propose_matches );
DECLARE( test_match_ann );
DECLARE( test_refine );
DECLARE( test_tracks );
DECLARE( test_initial_recon );
DECLARE( test_pipeline );

void register_tests()
{
  REGISTER( test_bundler_utils );
  REGISTER( test_tracks_detect );
  REGISTER( test_propose_matches );
  REGISTER( test_match_ann );
  REGISTER( test_refine );
  REGISTER( test_tracks );
  REGISTER( test_initial_recon );
  REGISTER( test_pipeline );
}

DEFINE_MAIN;
