#include <vcl_new.h>
#include <vcl_cstdio.h> // do not use iostream within operator new - it causes infinite recursion
#include <vcl_cstdlib.h>
#include <vcl_cstddef.h> // for vcl_size_t

#include <vnl/vnl_test.h>

#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_linear_operators_3.h>

bool verbose_malloc = false;
int malloc_count = 0;

// FIXME: Win32 will have different operator new in vnl dll from
// the one generated here, so this test fails - RWMC.
#ifndef WIN32
# define reset_count malloc_count = 0
# define check_count if (malloc_count<=1) vcl_printf("mallocs - PASSED\n"); else vcl_printf("mallocs - FAILED\t***\n")
#else
# define reset_count malloc_count = 0
# define check_count /* */
#endif

void test_matrix_fixed()
{
  verbose_malloc = true;
  double datablock[9] = {
    11, 12, 13,
    21, 22, 23,
    31, 32, 33,
  };

  vcl_printf("Calling ctor -- should be no mallocs\n");
  //Refvnl_double_3x3 X(datablock);
  reset_count;
  vnl_double_3x3 X(datablock);
  check_count;
  vcl_printf("X = [ %g %g %g\n      %g %g %g\n      %g %g %g ]\n",
             X(0,0),X(0,1),X(0,2),X(1,0),X(1,1),X(1,2),X(2,0),X(2,1),X(2,2));

  reset_count;
  vnl_double_3 v(10,11,12);
  check_count;
  vcl_printf("v = [ %g %g %g ]\n", v(0), v(1), v(2));
  
  reset_count;
  vnl_double_3 splork = X * (v + v);
  check_count;
  vcl_printf("splork = [ %g %g %g ]\n", splork(0), splork(1), splork(2));
  
  // This shouldn't compile...
  // vnl_matrix<double>* base = new vnl_double_3x3(datablock);

  vcl_printf("Now watch the mallocs\n");
  vnl_matrix<double>& CX = X;
  vnl_vector<double>& cv = v;
  vnl_vector<double> Xv = CX * (cv + cv);
  vcl_printf("X v = [ %g %g %g ]\n", Xv[0], Xv[1], Xv[2]);

  verbose_malloc = false;
}

// with gcc 3.0, formatted stream output uses operator
// new so printing to cout here causes stack overflow.

void* operator new(vcl_size_t s)
  // [18.4.1] lib.new.delete
#if defined(VCL_SUNPRO_CC_50) || defined(GNU_LIBSTDCXX_V3) || defined(VCL_KAI)
  throw (std::bad_alloc)
#endif
{
  void *r = vcl_malloc(s);

  ++malloc_count;

  if (verbose_malloc)
    vcl_printf("malloc: %08X for %d\n", unsigned(r), int(s));

  return r;
}

void operator delete(void* s)
#if defined(GNU_LIBSTDCXX_V3) || defined(VCL_SUNPRO_CC_50)
  throw ()
#endif
{
  if (verbose_malloc)
    vcl_printf("delete: %08X\n", unsigned(s));

  vcl_free(s);
}

TESTMAIN(test_matrix_fixed);
