#include <testlib/testlib_register.h>

DECLARE( test_scene );
DECLARE( test_cache );
DECLARE( test_io );
DECLARE( test_wrappers );
DECLARE( test_data );
DECLARE( test_block );



void register_tests()
{
  REGISTER( test_scene );
  REGISTER( test_cache );
  REGISTER( test_io );
  REGISTER( test_wrappers );
  REGISTER( test_data );
  REGISTER( test_block );

  
}


DEFINE_MAIN;
