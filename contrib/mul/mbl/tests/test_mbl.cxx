// First define testmain

#include <vnl/vnl_test.h>
#undef TESTMAIN
#define TESTMAIN(x)


#include <mbl/tests/test_stochastic_data_collector.cxx>
#include <mbl/tests/test_k_means.cxx>
#include <mbl/tests/test_mz_random.cxx>
#include <mbl/tests/test_matrix_products.cxx>
#include <mbl/tests/test_matxvec.cxx>
#include <mbl/tests/test_stats_1d.cxx>
#include <mbl/tests/test_sum_1d.cxx>

#undef TESTMAIN
#define TESTMAIN(x) int main() \
  { vnl_test_start(#x); x(); return vnl_test_summary(); }

void run_test_mbl()
{
  test_stochastic_data_collector();
  test_k_means();
  test_mz_random();
  test_matrix_products();
  test_matxvec();
  test_stats_1d();
  test_sum_1d();
}


TESTMAIN(run_test_mbl);
