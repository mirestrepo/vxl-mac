#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vnl/vnl_test.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vsl/vsl_binary_io.h>
#include <vnl/vnl_diag_matrix.h>
#include <vnl/io/vnl_io_diag_matrix.h>


void test_diag_matrix_double_io()
{
  vcl_cout << "*************************" << vcl_endl;
  vcl_cout << "test_diag_matrix_io"       << vcl_endl;
  vcl_cout << "*************************" << vcl_endl;
  //// test constructors, accessors
  const int n = 50;
  vnl_vector<double> v_out(n), v_in(n);

  for (int i=0; i<n; i++)
  {
    v_in(i) = (double)(i); // Different to check things change
    v_out(i) = (double)(i*i);
  }

  vnl_diag_matrix<double> diag_mat_out(v_out), diag_mat_in(v_in);

  vsl_print_summary(vcl_cout, diag_mat_out);
  vcl_cout << vcl_endl;

  vsl_b_ofstream bfs_out("vnl_diag_matrix_test_io.bvl.tmp");
  TEST ("Created vnl_diag_matrix_test_io.bvl.tmp for writing",
        (!bfs_out), false);
  vsl_b_write(bfs_out, diag_mat_out);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vnl_diag_matrix_test_io.bvl.tmp");
  TEST ("Opened vnl_diag_matrix_test_io.bvl.tmp for reading",
        (!bfs_in), false);
  vsl_b_read(bfs_in, diag_mat_in);
  bfs_in.close();

  TEST ("diag_mat_out.diagonal() == diag_mat_in.diagonal()",
        diag_mat_out.diagonal() == diag_mat_in.diagonal(), true);


  vsl_print_summary(vcl_cout, diag_mat_out);
  vcl_cout << vcl_endl;
}

#if 0
void test_vector()
{
  vnl_vector_test_double_io();
}

TESTMAIN(test_vector);
#endif
