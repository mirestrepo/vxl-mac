//:
// \file
#include <testlib/testlib_test.h>
#include <bvgl/bvgl_articulated_poly.h>
#include <vcl_iostream.h>

//: Test the articulated poly
static void test_articulated_poly()
{
#if 0 // default constr. not allowed to be used - GT
  bvgl_articulated_poly a;
#endif
  vcl_cout << "Not implemented yet\n";
  TEST("Test articulated poly ", 1, 1);
}

TESTMAIN( test_articulated_poly );
