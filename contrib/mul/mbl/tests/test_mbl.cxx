// First define testmain

#include <vnl/vnl_test.h>
#undef TESTMAIN
#define TESTMAIN(x)

#include <mbl/tests/test_arb_length_int.cxx>
#include <mbl/tests/test_selected_data_wrapper.cxx>
#include <mbl/tests/test_stochastic_data_collector.cxx>
#include <mbl/tests/test_histogram.cxx>
#include <mbl/tests/test_k_means.cxx>
#include <mbl/tests/test_mz_random.cxx>
#include <mbl/tests/test_matrix_products.cxx>
#include <mbl/tests/test_matxvec.cxx>
#include <mbl/tests/test_stats_1d.cxx>
#include <mbl/tests/test_sum_1d.cxx>
#include <mbl/tests/test_priority_bounded_queue.cxx>
#include <mbl/tests/test_gamma.cxx>
#include <mbl/tests/test_index_sort.cxx>
#include <mbl/tests/test_lru_cache.cxx>
#include <mbl/tests/test_thin_plate_spline_2d.cxx>
#include <mbl/mbl_print.h>

#undef TESTMAIN
#define TESTMAIN(x) int main() \
  { vnl_test_start(#x); x(); return vnl_test_summary(); }

void run_test_mbl()
{
  test_arb_length_int();
  test_selected_data_wrapper();
  test_priority_bounded_queue();
  test_stochastic_data_collector();
  test_k_means();
  test_mz_random();
  test_matrix_products();
  test_matxvec();
  test_stats_1d();
  test_sum_1d();
  test_gamma();
  test_index_sort();
  test_lru_cache();
  test_thin_plate_spline_2d();
  test_histogram();
}

TESTMAIN(run_test_mbl);
